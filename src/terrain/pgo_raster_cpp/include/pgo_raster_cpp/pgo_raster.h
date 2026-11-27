#ifndef PGO_RASTER_CPP__PGO_RASTER_H_
#define PGO_RASTER_CPP__PGO_RASTER_H_

#ifdef __cplusplus
extern "C" {
#endif

/// Rasterize (N,3) world points into a grid, taking the per-cell median z.
///
/// Arguments mirror pgo_gridmap_projector.rasterizer.rasterize():
///   xyz        row-major (x, y, z) doubles, length 3*n
///   n_rows/cols, resolution, origin_x/y, length_x/y  grid geometry
///   z_min/z_max height gate (inclusive)
///   out_elev   n_rows*n_cols doubles, flat index = col*n_rows + row,
///              NaN where a cell got no points
///   out_counts n_rows*n_cols accepted-point counts
///   out_max_z  optional (may be NULL) n_rows*n_cols doubles: the highest
///              accepted z per cell, NaN where a cell got no points.  Used
///              by the projector's clearance layer (2026-11-14).
///
/// Returns 0 on success, -1 on bad arguments.
int pgo_rasterize_median(
  const double * xyz, long n,
  int n_rows, int n_cols, double resolution,
  double origin_x, double origin_y, double length_x, double length_y,
  double z_min, double z_max,
  double * out_elev, long * out_counts, double * out_max_z);

#ifdef __cplusplus
}
#endif

#endif  // PGO_RASTER_CPP__PGO_RASTER_H_
