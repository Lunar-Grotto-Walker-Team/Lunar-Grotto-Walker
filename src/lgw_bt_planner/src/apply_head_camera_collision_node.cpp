// apply_head_camera_collision_node.cpp
//
// 启动时一次性把 head 相机(Realsense D435)加进 MoveIt 的 planning scene
// world,作为虚拟 collision_object,让 arm MoveGroup 规划时自动绕开 head
// 相机包围盒。
//
// 启动时序:launch 保证本节点在 agx_arm_moveit/demo.launch.py 之后起,
// 否则 /apply_planning_scene 不可用,本节点会等 30 s 后自退(报 ERROR)。
//
// 包围盒尺寸 = Realsense D435 物理尺寸(9 × 2.5 × 2.5 cm)+ 5 cm 边距
//   → BOX [0.15, 0.07, 0.07] m
// 位置 = head_calibration.json 里 camera_head_color_optical_frame 在
// arm base_link 系下的原点(arm base_link,不是 Go2 base_link)。
//
// Apply 成功后节点自退(rclcpp::shutdown() → main 退出码 0)。
// 若 MoveIt 重启,需要重跑这条 sanity(launch wrapper 加 nohup 即可)。

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>

#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <nlohmann/json.hpp>

#include <chrono>
#include <fstream>
#include <memory>
#include <string>

namespace lgw_bt_planner
{
namespace
{
// Realsense D435 物理尺寸 (9 × 2.5 × 2.5 cm) + 5 cm 边距 →
// 长度方向 +5 cm + 1 cm 余量 ≈ 0.15 m;高度/深度方向 +4.5 cm ≈ 0.07 m。
constexpr double kBoxX = 0.15;
constexpr double kBoxY = 0.07;
constexpr double kBoxZ = 0.07;

// 等 /apply_planning_scene 服务的最大时长(秒),超过直接自退。
constexpr int kServiceWaitTimeoutSec = 30;
// 一次 apply 调用的最大耗时(毫秒)。
constexpr int kCallTimeoutMs = 5000;
// timer 周期(毫秒):常态 500 ms,服务 ready 后再调用。
constexpr int kTickPeriodMs = 500;
}  // namespace

class ApplyHeadCameraCollisionNode : public rclcpp::Node
{
public:
  explicit ApplyHeadCameraCollisionNode(
    const rclcpp::NodeOptions& opts = rclcpp::NodeOptions())
  : rclcpp::Node("head_camera_collision", opts)
  {
    // === 1. 读 head_calibration.json,失败则自退 t0 ===
    try {
      const std::string share_dir =
        ament_index_cpp::get_package_share_directory("lgw_bt_planner");
      const std::string calib_path =
        share_dir + "/calibration/head_calibration.json";

      std::ifstream ifs(calib_path);
      if (!ifs.is_open()) {
        throw std::runtime_error("cannot open " + calib_path);
      }
      const nlohmann::json j = nlohmann::json::parse(ifs);
      const auto& pos = j.at("position");
      if (!pos.is_array() || pos.size() != 3) {
        throw std::runtime_error("position must be 3-element array");
      }
      head_cam_x_ = pos[0].get<double>();
      head_cam_y_ = pos[1].get<double>();
      head_cam_z_ = pos[2].get<double>();

      RCLCPP_INFO(this->get_logger(),
        "loaded head calibration: pos=(%.4f, %.4f, %.4f) from %s",
        head_cam_x_, head_cam_y_, head_cam_z_, calib_path.c_str());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(),
        "failed to load head_calibration.json: %s — aborting", e.what());
      load_failed_ = true;
      // 100 ms 后自退,让 launch wrapper 的启动日志有时间被 .run/*.log 接住
      timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() { rclcpp::shutdown(); });
      return;
    }

    // === 2. /apply_planning_scene 客户端 + 独立 callback group ===
    //
    // 2026-10-09 修复(见 memory entry):client 必须挂到独立的
    // MutuallyExclusive group,handler 里用本地 SingleThreadedExecutor
    // 只 add_callback_group(group_, node_base_)。否则
    //   rclcpp::spin_until_future_complete(node_base, fut, …)
    // 内部会 new 一个 executor 把整个 node add 进去,与 main 里
    // rclcpp::spin(node) 撞同一 node,抛 "already added to an executor"。
    apply_cb_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
    apply_client_ = this->create_client<moveit_msgs::srv::ApplyPlanningScene>(
      "/apply_planning_scene",
      rmw_qos_profile_services_default,
      apply_cb_group_);

    start_time_ = this->now();
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(kTickPeriodMs),
      std::bind(&ApplyHeadCameraCollisionNode::tick, this));

    RCLCPP_INFO(this->get_logger(),
      "waiting for /apply_planning_scene (timeout %d s, "
      "tick_period %d ms)",
      kServiceWaitTimeoutSec, kTickPeriodMs);
  }

private:
  void tick()
  {
    if (applied_ || load_failed_) {
      timer_->cancel();
      rclcpp::shutdown();
      return;
    }

    if (!apply_client_->service_is_ready()) {
      const double elapsed = (this->now() - start_time_).seconds();
      if (elapsed > static_cast<double>(kServiceWaitTimeoutSec)) {
        RCLCPP_ERROR(this->get_logger(),
          "/apply_planning_scene NOT READY after %.1f s — aborting "
          "(is agx_arm_moveit/demo.launch.py up?)",
          elapsed);
        timer_->cancel();
        rclcpp::shutdown();
        return;
      }
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "waiting for /apply_planning_service (%.1f s elapsed)…",
        elapsed);
      return;
    }

    // === 构造 request ===
    auto req = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();

    moveit_msgs::msg::CollisionObject co;
    co.header.frame_id = "base_link";  // arm base_link
    co.header.stamp = this->now();
    co.id = "head_camera";
    co.operation = moveit_msgs::msg::CollisionObject::ADD;

    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {kBoxX, kBoxY, kBoxZ};

    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = head_cam_x_;
    box_pose.position.y = head_cam_y_;
    box_pose.position.z = head_cam_z_;
    box_pose.orientation.w = 1.0;  // identity(轴对齐包围盒,简化)

    co.primitives.push_back(box);
    co.primitive_poses.push_back(box_pose);

    req->scene.world.collision_objects.push_back(co);
    req->scene.is_diff = true;  // 增量 add,不冲掉其他 collision_objects

    auto fut = apply_client_->async_send_request(req);

    // 同步等结果(本地 executor 只 add_callback_group,不 add_node)
    rclcpp::executors::SingleThreadedExecutor local_exec;
    local_exec.add_callback_group(
      apply_cb_group_, this->get_node_base_interface());
    const auto deadline = std::chrono::milliseconds(kCallTimeoutMs);
    const auto status =
      local_exec.spin_until_future_complete(fut, deadline);

    if (status != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_WARN(this->get_logger(),
        "apply_planning_scene call status=%d (NOT SUCCESS) — will retry "
        "on next tick",
        static_cast<int>(status));
      return;
    }

    auto resp = fut.get();
    if (!resp->success) {
      RCLCPP_ERROR(this->get_logger(),
        "apply_planning_scene returned success=false — aborting");
      timer_->cancel();
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(this->get_logger(),
      "OK: head camera collision object applied at base_link="
      "(%.4f, %.4f, %.4f) box=[%.3f, %.3f, %.3f] m, is_diff=true. "
      "Verify with: ros2 service call /get_planning_scene "
      "moveit_msgs/srv/GetPlanningScene \"{components: {components: 1}}\"",
      head_cam_x_, head_cam_y_, head_cam_z_, kBoxX, kBoxY, kBoxZ);
    applied_ = true;
    timer_->cancel();
    rclcpp::shutdown();
  }

  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_client_;
  rclcpp::CallbackGroup::SharedPtr apply_cb_group_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};

  double head_cam_x_{0.0};
  double head_cam_y_{0.0};
  double head_cam_z_{0.0};
  bool load_failed_{false};
  bool applied_{false};
};

}  // namespace lgw_bt_planner

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node =
    std::make_shared<lgw_bt_planner::ApplyHeadCameraCollisionNode>();
  rclcpp::spin(node);
  // 节点 tick() 里已 rclcpp::shutdown() 触发,这里再 shutdown 是 no-op。
  rclcpp::shutdown();
  return 0;
}