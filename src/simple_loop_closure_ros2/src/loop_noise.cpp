#include "simple_loop_closure/loop_noise.hpp"

#include <algorithm>

Eigen::VectorXd simple_loop_closure::makeLoopVariances(
    double fitness_score, double loop_var_scale, double loop_var_min,
    bool constrain_z, bool constrain_attitude, double free_var) {
  const double planar = std::max(loop_var_min, fitness_score * loop_var_scale);
  Eigen::VectorXd v = Eigen::VectorXd::Constant(6, planar);
  if (!constrain_attitude) {
    v(0) = free_var;  // Rx roll
    v(1) = free_var;  // Ry pitch
  }
  if (!constrain_z) {
    v(5) = free_var;  // Tz
  }
  return v;
}

double simple_loop_closure::makeLoopGate(double path_len, double abs_floor,
                                         double ratio, double cap) {
  double gate = std::max(abs_floor, ratio * std::abs(path_len));
  if (cap > 0.0) {
    gate = std::min(gate, cap);
  }
  return gate;
}
