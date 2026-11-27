// include/lgw_bt_planner/lookat_solver.hpp
//
// One interface for every lookat setpoint implementation. The solver is
// chosen ONCE at startup (lookat_solver: dls | qp); there is no runtime
// switching and no per-tick fallback (decision 2026-11-05). Both
// implementations return the same PostureResult, so the node, the
// preposition criteria and the control law are solver-agnostic.

#ifndef LGW_BT_PLANNER__LOOKAT_SOLVER_HPP_
#define LGW_BT_PLANNER__LOOKAT_SOLVER_HPP_

#include <cmath>
#include <string>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

#include "lgw_bt_planner/lookat_types.hpp"

namespace lgw_bt_planner
{

class LookatSolver
{
public:
  virtual ~LookatSolver() = default;

  // `p_target` is the target point in the arm base frame. The result's
  // tcp_desired is derived from the camera pose with the hand-eye transform
  // removed, identical to what the DLS family returns.
  virtual PostureResult solve(
    const tf2::Vector3 & p_target,
    const tf2::Matrix3x3 & R_cam_in_tip,
    const tf2::Vector3 & t_cam_in_tip,
    const PostureParams & params) = 0;
};

enum class SolverKind
{
  kDls,
  kQp,
};

// "dls" -> kDls, "qp" -> kQp, anything else -> `fallback` (the node logs a
// warning and keeps DLS so a typo cannot take the arm offline at startup).
SolverKind parse_solver_kind(const std::string & value, SolverKind fallback);

// 2026-11-19: 相机不许"上下倒转"。
//
// 推导(相机是 ROS optical 约定: z 前、x 右、y 下):
//   * 正立时, 相机 y 轴(图像"下")应当朝世界下方, 即 y_des = normalize(-z_w)
//     投影到垂直于光轴 u 的平面; x_des = y_des x u 就是图像右向。
//   * 光轴 u = +x 时代入: y_des = -z_w, x_des = (-z_w) x x_w = -y_w。
//   * 而求解器里 roll 项的参考量是 z_w x u = +y_w —— 与 x_des 恰好反向。
//     所以"正立"对应  R_cam.x . (z_w x u) <= 0, 滚 180 度时变成 > 0。
//
// 求解器里的 roll 项是 e = R_cam.x . (u x x_des) ∝ sin(roll): 它在 roll=0 和
// roll=180 处都为 0, 本质上是"软权重 + 二义极小", 无法排除倒转解。这个函数
// 用 cos(roll) 的符号做硬判据, 供两个求解器在接收解时调用。
//
// 退化情况: 光轴接近竖直(|z_w x u| 很小)时图像右向没有定义(绕光轴怎么滚都
// 能对准目标), 此时不判定, 直接视为通过。
inline bool camera_upright(
  const tf2::Vector3 & cam, const tf2::Vector3 & target,
  const tf2::Matrix3x3 & R_cam, double min_cos = 0.0)
{
  const tf2::Vector3 raw = target - cam;
  if (!std::isfinite(raw.length()) || raw.length() < 1e-6) {
    return true;
  }
  const tf2::Vector3 u = raw.normalized();
  tf2::Vector3 ref = tf2::Vector3(0.0, 0.0, 1.0).cross(u);   // = -x_des
  const double len = ref.length();
  if (len < 1e-3) {
    return true;  // 光轴接近竖直: roll 无定义
  }
  ref /= len;
  // ref = -x_des  =>  正立 <=> R_cam.x . ref <= -min_cos
  return R_cam.getColumn(0).dot(ref) <= -min_cos;
}

}  // namespace lgw_bt_planner

#endif  // LGW_BT_PLANNER__LOOKAT_SOLVER_HPP_
