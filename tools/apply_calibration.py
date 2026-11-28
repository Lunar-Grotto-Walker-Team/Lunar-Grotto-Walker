#!/usr/bin/env python3
"""Convert camera_calibration output (ost.yaml, OpenCV FileStorage format) to
the ROS2 CameraInfo YAML schema used by go2_front_camera_driver.

Usage:
    python3 apply_calibration.py <ost.yaml> <output.yaml>

Reads:
    image_width, image_height, camera_matrix (3x3), distortion_coefficients (1x5),
    distortion_model.

Writes:
    ROS2-style YAML with rows/cols/data blocks for K/D/R/P plus camera_name set to
    "go2_front_camera" (overriding whatever the calibrator recorded).

Requires:
    python3-yaml (apt: ros-humble-camera-calibration dependency, or python3-yaml).

Does NOT require OpenCV or ROS Python — pure stdlib + PyYAML so this works in any
container that has python3-yaml.
"""

import argparse
import re
import sys
from pathlib import Path

import yaml


# OpenCV FileStorage YAML uses !!opencv-matrix with rows/cols/dt/data.
# PyYAML's SafeLoader rejects unknown tags; we strip the tag and let the inner
# keys (rows/cols/dt/data) parse as a normal mapping.
_OPENCV_MATRIX_TAG = re.compile(r"!!opencv-matrix")
_YAML_DIRECTIVE = re.compile(r"^%YAML:[0-9.]+\s*\n", re.MULTILINE)


def _flatten_opencv_matrix(node):
    """Recursively replace {rows, cols, data} dicts with their flat data list."""
    if isinstance(node, dict):
        if {"rows", "cols", "data"} <= set(node.keys()):
            return list(node["data"])
        return {k: _flatten_opencv_matrix(v) for k, v in node.items()}
    if isinstance(node, list):
        return [_flatten_opencv_matrix(x) for x in node]
    return node


def _read_ost_yaml(path: Path) -> dict:
    raw = path.read_text()
    raw = _YAML_DIRECTIVE.sub("", raw)
    raw = _OPENCV_MATRIX_TAG.sub("", raw)
    return yaml.safe_load(raw)


def _format_floats(values, precision: int = 6) -> str:
    parts = []
    for v in values:
        if isinstance(v, int):
            parts.append(f"{float(v):.{precision}f}")
        else:
            parts.append(f"{float(v):.{precision}f}")
    return ", ".join(parts)


def ost_to_camera_info_yaml(
    ost_path: Path,
    out_path: Path,
    camera_name: str = "go2_front_camera",
    precision: int = 6,
) -> None:
    parsed = _read_ost_yaml(ost_path)

    if "image_width" not in parsed or "image_height" not in parsed:
        raise SystemExit(f"{ost_path}: missing image_width / image_height")
    if "camera_matrix" not in parsed:
        raise SystemExit(f"{ost_path}: missing camera_matrix")
    if "distortion_coefficients" not in parsed:
        raise SystemExit(f"{ost_path}: missing distortion_coefficients")

    image_width = int(parsed["image_width"])
    image_height = int(parsed["image_height"])

    K = _flatten_opencv_matrix(parsed["camera_matrix"])
    D = _flatten_opencv_matrix(parsed["distortion_coefficients"])
    distortion_model = parsed.get("distortion_model") or "plumb_bob"

    if len(K) != 9:
        raise SystemExit(f"{ost_path}: camera_matrix expected 9 values, got {len(K)}")
    if len(D) not in (4, 5, 8, 12, 14):
        raise SystemExit(
            f"{ost_path}: distortion_coefficients length {len(D)} not in "
            f"{{4, 5, 8, 12, 14}} (plumb_bob / rational_polynomial / equidistant)"
        )

    # Mono camera: rectification = identity, projection_matrix = [K | 0].
    R = [1.0, 0.0, 0.0,  0.0, 1.0, 0.0,  0.0, 0.0, 1.0]
    P = [
        K[0], 0.0,  K[2], 0.0,
        0.0,  K[4], K[5], 0.0,
        0.0,  0.0,  1.0,  0.0,
    ]

    # Pad / truncate distortion_coefficients to 5 entries (plumb_bob standard).
    # For other models (rational_polynomial, equidistant) the camera_info_manager
    # loader reads however many coefficients the model expects, but keeping the
    # file at 5 entries is the most portable.
    D_padded = list(D) + [0.0] * (5 - len(D))
    D_padded = D_padded[:5]

    lines = [
        f"image_width: {image_width}",
        f"image_height: {image_height}",
        f'camera_name: "{camera_name}"',
        "",
        "camera_matrix:",
        "  rows: 3",
        "  cols: 3",
        f"  data: [{_format_floats(K, precision)}]",
        "",
        f"distortion_model: {distortion_model}",
        "distortion_coefficients:",
        "  rows: 1",
        "  cols: 5",
        f"  data: [{_format_floats(D_padded, precision)}]",
        "",
        "rectification_matrix:",
        "  rows: 3",
        "  cols: 3",
        f"  data: [{_format_floats(R, precision)}]",
        "",
        "projection_matrix:",
        "  rows: 3",
        "  cols: 4",
        f"  data: [{_format_floats(P, precision)}]",
        "",
    ]
    out_path.write_text("\n".join(lines))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    parser.add_argument("ost_yaml", type=Path, help="Input ost.yaml from camera_calibration")
    parser.add_argument("output_yaml", type=Path, help="Output ROS2 CameraInfo YAML path")
    parser.add_argument(
        "--camera-name",
        default="go2_front_camera",
        help="camera_name to set in output (default: go2_front_camera)",
    )
    parser.add_argument(
        "--precision",
        type=int,
        default=6,
        help="Decimal places for floats in output (default: 6)",
    )
    args = parser.parse_args(argv)

    ost_to_camera_info_yaml(
        args.ost_yaml,
        args.output_yaml,
        camera_name=args.camera_name,
        precision=args.precision,
    )
    print(f"[apply_calibration] Wrote {args.output_yaml}")


if __name__ == "__main__":
    main()
