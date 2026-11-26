// Reference generator for the GridMap layout.
//
// It replicates what grid_map::GridMapRosConverter::fromMessage does, so
// it can be built against grid_map_core alone (no ROS msg headers):
//   setGeometry(Length(length_x, length_y), resolution, Position(pose))
//   m(r, c) = data[c * n_rows + r]      (column-major Eigen::Map)
//   setStartIndex(Index(outer_start_index, inner_start_index))
//
// It then asks grid_map where world (0.37, -12.40) belongs and reports
// that cell together with the flat index of a sentinel placed there.
//
// Build (inside the container, from a scratch ament workspace so the
// static lib resolves):
//   g++ -O0 -o /tmp/gm_ref test/gridmap_reference.cpp \
//     -I<elevmap_fastlio>/install/grid_map_core/include/grid_map_core \
//     -I/usr/include/eigen3 \
//     -L<elevmap_fastlio>/install/grid_map_core/lib -lgrid_map_core
//
// Note: read the flat array as RAW BINARY. A .npy file has a ~128-byte
// header, which silently shifts every element by 32 floats and makes the
// comparison fail with a baffling 32-element offset.

#include <grid_map_core/grid_map_core.hpp>
#include <cstdio>
#include <fstream>
#include <vector>
#include <limits>

int main() {
  const int N = 500;
  const double RES = 0.10;
  const double LEN = 50.0;

  grid_map::GridMap gm;
  gm.setGeometry(grid_map::Length(LEN, LEN), RES, grid_map::Position(0.0, 0.0));
  gm.setStartIndex(grid_map::Index(0, 0));

  // Where does grid_map say world (0.37, -12.40) belongs?
  grid_map::Index idx;
  gm.getIndex(grid_map::Position(0.37, -12.40), idx);

  // Lay out a matrix the way fromMessage does, with a sentinel in that cell.
  grid_map::Matrix m(N, N);
  m.setConstant(std::numeric_limits<float>::quiet_NaN());
  m(idx(0), idx(1)) = 12345.0f;
  gm.add("elevation", m);

  const grid_map::Matrix & mm = gm.get("elevation");
  long flat = -1;
  for (long i = 0; i < (long)mm.size(); i++) {
    if (mm.data()[i] == 12345.0f) { flat = i; break; }
  }

  printf("n_rows %ld\n", (long)mm.rows());
  printf("n_cols %ld\n", (long)mm.cols());
  printf("row %ld\n", (long)idx(0));
  printf("col %ld\n", (long)idx(1));
  printf("flat %ld\n", flat);
  printf("is_row_major %d\n", (int)mm.IsRowMajor);
  printf("start_index %ld %ld\n",
         (long)gm.getStartIndex()(0), (long)gm.getStartIndex()(1));
  return 0;
}
