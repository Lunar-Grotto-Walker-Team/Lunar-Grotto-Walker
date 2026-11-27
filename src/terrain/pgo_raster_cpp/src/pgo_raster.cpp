// Native per-cell median rasterizer for pgo_gridmap_projector.
//
// The Python implementation sorts (cell, z) pairs with np.lexsort, which is
// O(N log N) and costs ~590 ms for the 846k-point maps this stack produces.
// Here the cell grouping is a counting sort (O(N)) and the median is
// std::nth_element per cell (also O(N) overall), with cells distributed over
// OpenMP threads.
//
// The result MUST match rasterize(..., aggregation="median") cell for cell,
// including the even-count rule (average of the two middle values) and the
// NaN sentinel for empty cells.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "pgo_raster_cpp/pgo_raster.h"

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

}  // namespace

extern "C" int pgo_rasterize_median(
  const double * xyz, long n,
  int n_rows, int n_cols, double resolution,
  double origin_x, double origin_y, double length_x, double length_y,
  double z_min, double z_max,
  double * out_elev, long * out_counts, double * out_max_z)
{
  if (xyz == nullptr || out_elev == nullptr || out_counts == nullptr) {
    return -1;
  }
  if (n_rows <= 0 || n_cols <= 0 || resolution <= 0.0) {
    return -1;
  }

  const long n_cells = static_cast<long>(n_rows) * n_cols;
  const double half_x = 0.5 * length_x;
  const double half_y = 0.5 * length_y;

  std::fill(out_elev, out_elev + n_cells, kNaN);
  std::fill(out_counts, out_counts + n_cells, 0L);
  if (out_max_z != nullptr) {
    std::fill(out_max_z, out_max_z + n_cells, kNaN);
  }
  if (n <= 0) {
    return 0;
  }

  // Counting sort by cell.  Two passes over the points, one scatter pass.
  for (long i = 0; i < n; ++i) {
    const double x = xyz[3 * i + 0];
    const double y = xyz[3 * i + 1];
    const double z = xyz[3 * i + 2];
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }
    if (z < z_min || z > z_max) {
      continue;
    }
    const long row = static_cast<long>(
      std::floor((origin_x + half_x - x) / resolution));
    const long col = static_cast<long>(
      std::floor((origin_y + half_y - y) / resolution));
    if (row < 0 || row >= n_rows || col < 0 || col >= n_cols) {
      continue;
    }
    ++out_counts[col * n_rows + row];
  }

  // Prefix sums (starts) and a scatter cursor.
  std::vector<long> starts(static_cast<size_t>(n_cells) + 1, 0L);
  for (long c = 0; c < n_cells; ++c) {
    starts[static_cast<size_t>(c) + 1] =
      starts[static_cast<size_t>(c)] + out_counts[c];
  }
  const long accepted = starts[static_cast<size_t>(n_cells)];
  if (accepted == 0) {
    return 0;
  }

  std::vector<double> values(static_cast<size_t>(accepted));
  std::vector<long> cursor(starts.begin(), starts.end() - 1);
  for (long i = 0; i < n; ++i) {
    const double x = xyz[3 * i + 0];
    const double y = xyz[3 * i + 1];
    const double z = xyz[3 * i + 2];
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }
    if (z < z_min || z > z_max) {
      continue;
    }
    const long row = static_cast<long>(
      std::floor((origin_x + half_x - x) / resolution));
    const long col = static_cast<long>(
      std::floor((origin_y + half_y - y) / resolution));
    if (row < 0 || row >= n_rows || col < 0 || col >= n_cols) {
      continue;
    }
    const long cell = col * n_rows + row;
    values[static_cast<size_t>(cursor[static_cast<size_t>(cell)]++)] = z;
  }

  // Per-cell median.  Matches numpy: mid = (count - 1) / 2, and for an even
  // count average the two middle values.
#ifdef _OPENMP
#pragma omp parallel for schedule(static)
#endif
  for (long cell = 0; cell < n_cells; ++cell) {
    const long count = out_counts[cell];
    if (count <= 0) {
      continue;
    }
    double * begin = values.data() + starts[static_cast<size_t>(cell)];
    double * end = begin + count;
    const long mid = (count - 1) / 2;
    if (count % 2 == 1) {
      std::nth_element(begin, begin + mid, end);
      out_elev[cell] = begin[mid];
    } else {
      // numpy: mid = (count - 1) / 2, average sorted_z[mid] and
      // sorted_z[mid + 1].  Put the (mid + 1)-th smallest at begin[mid + 1];
      // the lower middle is then the maximum of the part before it.
      const long upper_idx = mid + 1;
      std::nth_element(begin, begin + upper_idx, end);
      const double upper = begin[upper_idx];
      const double lower = *std::max_element(begin, begin + upper_idx);
      out_elev[cell] = 0.5 * (lower + upper);
    }
  }
  if (out_max_z != nullptr) {
#ifdef _OPENMP
    #pragma omp parallel for schedule(static)
#endif
    for (long cell = 0; cell < n_cells; ++cell) {
      const long count = out_counts[cell];
      if (count > 0) {
        double best = -std::numeric_limits<double>::infinity();
        for (long k = 0; k < count; ++k) {
          const double v = values[static_cast<size_t>(starts[cell]) + k];
          best = v > best ? v : best;
        }
        out_max_z[cell] = best;
      }
    }
  }

  return 0;
}

// ---------------------------------------------------------------------------
// Per-cell z sort for the per-cell ground reference (ground.py::cell_stats).
//
// The Python side used np.lexsort((z, cell)) to group points by coarse cell
// and sort z inside each group: 1.08 s for 1.5M points, 82% of the whole
// reference build, which capped reference mode at 0.5 Hz.  Same counting-sort
// trick as the median kernel, plus a std::sort inside each (small) cell.
//
// out_counts / out_first are n_cells long; out_first is -1 for empty cells.
// out_z must have room for n entries (accepted count <= n).
// ---------------------------------------------------------------------------
extern "C" int pgo_cell_sort(
  const double * xyz, long n,
  int n_rows, int n_cols, double resolution,
  double origin_x, double origin_y, double length_x, double length_y,
  long * out_counts, long * out_first, double * out_z)
{
  if (xyz == nullptr || out_counts == nullptr || out_first == nullptr ||
    out_z == nullptr)
  {
    return -1;
  }
  if (n_rows <= 0 || n_cols <= 0 || resolution <= 0.0) {
    return -1;
  }

  const long n_cells = static_cast<long>(n_rows) * n_cols;
  const double half_x = 0.5 * length_x;
  const double half_y = 0.5 * length_y;

  std::fill(out_counts, out_counts + n_cells, 0L);
  std::fill(out_first, out_first + n_cells, -1L);
  if (n <= 0) {
    return 0;
  }

  // Pass 1: count.
  for (long i = 0; i < n; ++i) {
    const double x = xyz[3 * i + 0];
    const double y = xyz[3 * i + 1];
    const double z = xyz[3 * i + 2];
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }
    const long row = static_cast<long>(
      std::floor((origin_x + half_x - x) / resolution));
    const long col = static_cast<long>(
      std::floor((origin_y + half_y - y) / resolution));
    if (row < 0 || row >= n_rows || col < 0 || col >= n_cols) {
      continue;
    }
    ++out_counts[col * n_rows + row];
  }

  // Starts (first index of each cell in out_z) and a scatter cursor.
  long acc = 0;
  std::vector<long> cursor(static_cast<size_t>(n_cells), 0L);
  for (long c = 0; c < n_cells; ++c) {
    if (out_counts[c] > 0) {
      out_first[c] = acc;
      cursor[static_cast<size_t>(c)] = acc;
      acc += out_counts[c];
    }
  }
  if (acc == 0) {
    return 0;
  }

  // Pass 2: scatter z.
  for (long i = 0; i < n; ++i) {
    const double x = xyz[3 * i + 0];
    const double y = xyz[3 * i + 1];
    const double z = xyz[3 * i + 2];
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }
    const long row = static_cast<long>(
      std::floor((origin_x + half_x - x) / resolution));
    const long col = static_cast<long>(
      std::floor((origin_y + half_y - y) / resolution));
    if (row < 0 || row >= n_rows || col < 0 || col >= n_cols) {
      continue;
    }
    const long cell = col * n_rows + row;
    out_z[cursor[static_cast<size_t>(cell)]++] = z;
  }

  // Pass 3: sort z inside each cell (cells hold a few dozen points).
#ifdef _OPENMP
  #pragma omp parallel for schedule(static)
#endif
  for (long c = 0; c < n_cells; ++c) {
    const long cnt = out_counts[c];
    if (cnt > 1) {
      std::sort(out_z + out_first[c], out_z + out_first[c] + cnt);
    }
  }
  return 0;
}

// ---------------------------------------------------------------------------
// Per-cell ground reference flood (ground.py::_flood_python).
//
// The Python version keeps the BFS state in lists and looks the z band up
// with bisect; on a 70 m window that is still ~170 ms per frame and caps
// reference mode below its 2 Hz target.  This is the same algorithm in C++:
// FIFO over a growing vector, 8 neighbours in the same order, and a binary
// search inside the already-sorted per-cell slice.
//
// ref / fill / accepted / gap are mutated in place.  count == 0 cells may be
// accepted with the neighbour's reference while gap allows it; cells with
// points are accepted only when at least min_points_ref of them fall in
// [ref - band_below, ref + band_above].
// ---------------------------------------------------------------------------
namespace {
const int kNeighbours[8][2] = {
  {-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1}};

// std::lower_bound / upper_bound over sorted_z[start, end).
long lower_bound_idx(const double * z, long start, long end, double x)
{
  return static_cast<long>(
    std::lower_bound(z + start, z + end, x) - z);
}

long upper_bound_idx(const double * z, long start, long end, double x)
{
  return static_cast<long>(
    std::upper_bound(z + start, z + end, x) - z);
}
}  // namespace

extern "C" int pgo_ground_flood(
  int n_rows, int n_cols,
  const long * counts, const long * first, const double * sorted_z,
  const long * seed_cells, long n_seed,
  double band_below, double band_above, int min_points_ref,
  double p_frac, int max_gap_cells,
  int slope_exception_enabled, double upright_spread_max,
  double slope_except_above,
  const double * p20, const double * p50, const double * p80,
  double * ref, double * fill, unsigned char * accepted, long * gap)
{
  if (n_rows <= 0 || n_cols <= 0 || counts == nullptr || first == nullptr ||
    sorted_z == nullptr || ref == nullptr || fill == nullptr ||
    accepted == nullptr || gap == nullptr)
  {
    return -1;
  }
  if (n_seed > 0 && seed_cells == nullptr) {
    return -1;
  }

  const long n_cells = static_cast<long>(n_rows) * n_cols;
  std::vector<long> queue(seed_cells, seed_cells + n_seed);
  size_t head = 0;

  while (head < queue.size()) {
    const long cur = queue[head++];
    const long row = cur % n_rows;
    const long col = cur / n_rows;
    const double ref_nb = ref[cur];
    const double fill_nb = fill[cur];
    const long gap_nb = gap[cur];

    for (int k = 0; k < 8; ++k) {
      const long rr = row + kNeighbours[k][0];
      const long cc = col + kNeighbours[k][1];
      if (rr < 0 || rr >= n_rows || cc < 0 || cc >= n_cols) {
        continue;
      }
      const long nb = cc * n_rows + rr;
      if (accepted[nb]) {
        continue;
      }
      const long n = counts[nb];
      if (n == 0) {
        if (gap_nb + 1 <= max_gap_cells) {
          ref[nb] = ref_nb;
          fill[nb] = fill_nb;
          gap[nb] = gap_nb + 1;
          accepted[nb] = 1;
          queue.push_back(nb);
        }
        continue;
      }

      const long start = first[nb];
      const long end = start + n;
      const long lo = lower_bound_idx(sorted_z, start, end,
          ref_nb - band_below);
      const long hi = upper_bound_idx(sorted_z, lo, end,
          ref_nb + band_above);
      const long count = hi - lo;
      if (count >= min_points_ref) {
        ref[nb] = sorted_z[lo + static_cast<long>(p_frac * (count - 1))];
        const long mid = count / 2;
        fill[nb] = (count % 2 == 1)
          ? sorted_z[lo + mid]
          : 0.5 * (sorted_z[lo + mid - 1] + sorted_z[lo + mid]);
        gap[nb] = 0;
        accepted[nb] = 1;
        queue.push_back(nb);
        continue;
      }

      if (!slope_exception_enabled) {
        continue;
      }
      if (p80[nb] - p20[nb] > upright_spread_max) {
        continue;
      }
      if (std::fabs(p20[nb] - ref_nb) <= slope_except_above) {
        ref[nb] = p20[nb];
        fill[nb] = p50[nb];
        gap[nb] = 0;
        accepted[nb] = 1;
        queue.push_back(nb);
      }
    }
  }
  return 0;
}
