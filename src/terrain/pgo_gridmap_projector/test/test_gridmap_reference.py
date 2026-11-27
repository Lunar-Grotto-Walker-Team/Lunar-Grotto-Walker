"""Cross-check our GridMap layout against the real grid_map library.

The ground truth is produced by compiling and running `gridmap_reference.cpp`
against the real `grid_map_core` and asking it where a known world point
belongs.

Two rules make this test trustworthy:

  1. The C++ cross-check is the *only* accepted source of truth. If the
     toolchain, `grid_map_core` or the compiler is unavailable, the test
     SKIPS with an explicit "the C++ cross-check did NOT run" message.
     It never falls back to the committed golden file and reports success:
     a golden file that was never checked against the library is an
     assumption, not a verification.
  2. When the C++ program DOES run, its stdout is compared byte-for-byte
     against the committed golden file, so the golden cannot go stale.

The compile and the run both have timeouts, and the build output goes to a
unique temporary directory, so parallel pytest workers cannot race on the
same output path and a hung compile cannot hang the suite.
"""
import os
import shutil
import subprocess
import tempfile

import numpy as np
import pytest

from pgo_gridmap_projector.gridmap_msg import (
    GRIDMAP_COLUMN_MAJOR,
    build_gridmap_msg,
)
from pgo_gridmap_projector.rasterizer import GridSpec


HERE = os.path.dirname(os.path.abspath(__file__))
GOLDEN = os.path.join(HERE, "gridmap_reference_golden.txt")
SRC = os.path.join(HERE, "gridmap_reference.cpp")

# Deliberately 50 m, NOT the projector's current 30 m default. This test
# pins the index convention against a real grid_map_core compile, and the
# golden file was produced at this geometry (sentinel at (0.37, -12.40)
# landing in row 246 / col 373 / flat 186746). The grid size here is
# irrelevant to what is being verified; changing it would invalidate the
# golden numbers without testing anything more.
SPEC = GridSpec(resolution=0.10, length_x=50.0, length_y=50.0)

COMPILE_TIMEOUT_S = 180
RUN_TIMEOUT_S = 30


def _grid_map_core_prefix():
    """Locate an installed grid_map_core, or return None."""
    candidates = [
        "/my_ws/l4t/nav_ws/elevmap_fastlio/install/grid_map_core",
    ]
    for c in candidates:
        if os.path.isdir(os.path.join(c, "include", "grid_map_core")):
            return c
    return None


def _parse(text):
    out = {}
    for line in text.strip().splitlines():
        parts = line.split()
        # NOTE: the reference program prints "start_index 0 0" (three
        # tokens), so the value may carry more than one number. Keep the
        # remaining tokens joined; the caller splits them as needed.
        if len(parts) >= 2:
            out[parts[0]] = " ".join(parts[1:])
    return out


def _reference_from_cpp():
    """Compile and run the C++ reference program.

    Returns:
        ``(stdout_text, None)`` when the C++ path ran successfully, or
        ``(None, reason)`` when it could not be exercised (missing
        grid_map_core, missing compiler, compile error, timeout, ...).
        The reason is meant to be shown to the user in the skip message.
    """
    prefix = _grid_map_core_prefix()
    if prefix is None:
        return None, "grid_map_core was not found at any known prefix"

    # Unique build directory per process/run: parallel pytest workers must
    # not race on the same output path.
    workdir = tempfile.mkdtemp(prefix="gm_ref_%d_" % os.getpid())
    exe = os.path.join(workdir, "gm_ref_test")
    cmd = [
        "g++", "-O0", "-o", exe, SRC,
        "-I", os.path.join(prefix, "include", "grid_map_core"),
        "-I", "/usr/include/eigen3",
        "-L", os.path.join(prefix, "lib"),
        "-lgrid_map_core",
    ]
    try:
        try:
            subprocess.run(cmd, check=True, capture_output=True,
                           timeout=COMPILE_TIMEOUT_S)
        except FileNotFoundError:
            return None, "g++ was not found on PATH"
        except subprocess.TimeoutExpired:
            return None, ("compiling the reference program timed out after "
                          "%ds" % COMPILE_TIMEOUT_S)
        except subprocess.CalledProcessError as exc:
            err = (exc.stderr or b"").decode("utf-8", "replace").strip()
            return None, "compiling the reference program failed: %s" % err
        except OSError as exc:
            return None, "compiling the reference program failed: %s" % exc

        try:
            res = subprocess.run([exe], check=True, capture_output=True,
                                 text=True, timeout=RUN_TIMEOUT_S)
        except subprocess.TimeoutExpired:
            return None, ("running the reference program timed out after %ds"
                          % RUN_TIMEOUT_S)
        except subprocess.CalledProcessError as exc:
            err = (exc.stderr or "").strip()
            return None, "running the reference program failed: %s" % err
        return res.stdout, None
    finally:
        shutil.rmtree(workdir, ignore_errors=True)


def _read_golden():
    """Return the golden file's exact text, or None if it does not exist."""
    if not os.path.exists(GOLDEN):
        return None
    with open(GOLDEN, "rb") as handle:
        return handle.read().decode("utf-8")


@pytest.fixture(scope="module")
def cpp_reference():
    """The C++ reference program's stdout.

    Skips the whole module when the C++ cross-check cannot be run. The
    golden file is deliberately NOT used as a substitute: it is only ever
    compared against this output, never trusted on its own.
    """
    out, reason = _reference_from_cpp()
    if out is None:
        pytest.skip(
            "the C++ grid_map cross-check did NOT run (%s); the committed "
            "golden file is not a substitute for it and no verification of "
            "the layout was performed" % reason
        )
    return out


def test_cpp_reference_matches_golden_byte_for_byte(cpp_reference):
    """The golden file must be exactly what the real grid_map produces."""
    golden = _read_golden()
    assert golden is not None, (
        "golden reference file is missing: %s — regenerate it with the "
        "command in the task brief" % GOLDEN
    )
    assert cpp_reference == golden, (
        "the C++ reference output differs from the committed golden file; "
        "the golden is stale and must be regenerated:\n"
        "--- C++ ---\n%s\n--- golden ---\n%s" % (cpp_reference, golden)
    )


def test_layout_matches_grid_map_reference(cpp_reference):
    ref = _parse(cpp_reference)

    assert int(ref["n_rows"]) == 500
    assert int(ref["n_cols"]) == 500
    assert int(ref["start_index"].split()[0]) == 0
    assert int(ref["start_index"].split()[1]) == 0
    # grid_map's Matrix is Eigen column-major, which is exactly what our
    # GRIDMAP_COLUMN_MAJOR constant claims.
    assert ref["is_row_major"] == "0"
    assert GRIDMAP_COLUMN_MAJOR is True

    row, col = int(ref["row"]), int(ref["col"])
    flat = int(ref["flat"])

    # grid_map's flat index for cell (row, col) in a column-major buffer.
    assert flat == col * 500 + row

    elevation = np.full((500, 500), np.nan, dtype=np.float64)
    elevation[row, col] = 12345.0
    msg = build_gridmap_msg(elevation, SPEC, "map", 0)

    # Our flat index must equal grid_map's.
    assert msg.data[0].data[flat] == pytest.approx(12345.0)
    # And it must be the only non-NaN value.
    nonzero = [i for i, v in enumerate(msg.data[0].data)
               if not np.isnan(v)]
    assert nonzero == [flat]
