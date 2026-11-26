#pragma once

#include <Eigen/Dense>

namespace simple_loop_closure {

// Returns 6 variances in GTSAM Pose3 tangent order [Rx,Ry,Rz,Tx,Ty,Tz].
// A DOF whose constrain_* flag is false gets `free_var` (effectively a
// no-op against the odometry chain); the others get the ICP fitness
// variance with `loop_var_min` as a floor.
Eigen::VectorXd makeLoopVariances(double fitness_score, double loop_var_scale,
                                  double loop_var_min, bool constrain_z,
                                  bool constrain_attitude, double free_var);

// Consistency gate for a loop edge:
//   gate = max(abs_floor, ratio * path_len), capped at `cap` when cap > 0.
// The floor keeps short-baseline false loops out; the distance-proportional
// part lets a true ring closure carry the accumulated drift.  The cap is the
// defence the ratio cannot provide: in a self-similar corridor the axial ICP
// slide grows with the baseline, so an uncapped ratio lets a 45 m loop carry
// a 2.5 m slide (and a 148 m loop 8.9 m).  cap <= 0 disables the cap and
// restores the pre-2026-11-17 behaviour.
double makeLoopGate(double path_len, double abs_floor, double ratio,
                    double cap = 0.0);

}  // namespace simple_loop_closure
