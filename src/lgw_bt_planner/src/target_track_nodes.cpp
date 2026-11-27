// src/target_track_nodes.cpp — see the header for the contracts.

#include "lgw_bt_planner/target_track_nodes.hpp"

#include "lgw_bt_planner/target_track_query.hpp"

#include <rclcpp/logging.hpp>

#include <chrono>

namespace lgw_bt_planner
{

namespace
{
constexpr char kTrackTopic[] = "/perception/tracks/unified";
constexpr char kArmResultTopic[] = "/perception/two_stage/camera_arm/result";
}  // namespace

void TrackArrayCache::init(rclcpp::Node * node)
{
  sub = node->create_subscription<yoloe_interfaces::msg::UnifiedTrackArray>(
    kTrackTopic, rclcpp::QoS(10),
    [this](const yoloe_interfaces::msg::UnifiedTrackArray::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex);
      latest = *msg;
      seen = true;
    });
}

bool TrackArrayCache::snapshot(
  yoloe_interfaces::msg::UnifiedTrackArray * out) const
{
  std::lock_guard<std::mutex> lock(mutex);
  if (!seen) {
    return false;
  }
  *out = latest;
  return true;
}

// ---------------------------------------------------------------------------
// IsTargetAvailable (reactive condition)
// ---------------------------------------------------------------------------

BT::PortsList IsTargetAvailable::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<int>(
      "max_age_ms", 1000,
      "A selected track staler than this does not count as available"),
    BT::OutputPort<int64_t>("track_id", "Selected track id on SUCCESS"),
  });
}

BT::NodeStatus IsTargetAvailable::onTick(
  const std::shared_ptr<yoloe_interfaces::msg::UnifiedTrackArray> & last_msg)
{
  if (!last_msg) {
    return BT::NodeStatus::FAILURE;
  }
  const auto * track = track_query::selected_track(*last_msg);
  if (track == nullptr) {
    return BT::NodeStatus::FAILURE;
  }
  int max_age_ms = 1000;
  getInput("max_age_ms", max_age_ms);
  if (!track_query::is_fresh(*track, max_age_ms)) {
    return BT::NodeStatus::FAILURE;
  }
  setOutput("track_id", static_cast<int64_t>(track->track_id));
  return BT::NodeStatus::SUCCESS;
}

// ---------------------------------------------------------------------------
// IsTargetWithin (reactive condition, patrol preemption gate)
// ---------------------------------------------------------------------------

BT::PortsList IsTargetWithin::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<double>("min_xy_m", 0.0, "Lower bound of the XoY band (m)"),
    BT::InputPort<double>("max_xy_m", 0.55, "Upper bound of the XoY band (m)"),
    BT::InputPort<int>(
      "max_age_ms", 1000,
      "A selected track staler than this does not count"),
    BT::InputPort<double>(
      "max_azimuth_rad", 0.0,
      "Sector half-angle (rad); <=0 disables the azimuth criterion"),
    BT::InputPort<double>(
      "sector_center_rad", 0.0,
      "Sector centre azimuth in the arm base frame (rad); 0 = +X"),
    BT::OutputPort<int64_t>("track_id", "Selected track id when inside"),
  });
}

BT::NodeStatus IsTargetWithin::onTick(
  const std::shared_ptr<yoloe_interfaces::msg::UnifiedTrackArray> & last_msg)
{
  double min_xy = 0.0;
  double max_xy = 0.55;
  int max_age_ms = 1000;
  double max_az = 0.0;
  double sector_center = 0.0;
  getInput("min_xy_m", min_xy);
  getInput("max_xy_m", max_xy);
  getInput("max_age_ms", max_age_ms);
  getInput("max_azimuth_rad", max_az);
  getInput("sector_center_rad", sector_center);

  const auto fail = [this]() {
      if (inside_) {
        inside_ = false;
      }
      return BT::NodeStatus::FAILURE;
    };

  if (!last_msg) {
    return fail();
  }
  const auto * track = track_query::selected_track(*last_msg);
  if (track == nullptr || !track_query::is_fresh(*track, max_age_ms)) {
    return fail();
  }
  if (!track_query::xy_in_band(*track, min_xy, max_xy)) {
    return fail();
  }
  if (max_az > 0.0 && !track_query::in_front_sector(*track, max_az, sector_center)) {
    return fail();
  }

  if (!inside_) {
    inside_ = true;
    RCLCPP_INFO(
      logger(),
      "[%s] target id=%lu inside band (XoY=%.2f m, band [%.2f, %.2f]) "
      "-> preempt",
      name().c_str(), static_cast<unsigned long>(track->track_id),
      track_query::arm_base_xy_m(*track), min_xy, max_xy);
  }
  setOutput("track_id", static_cast<int64_t>(track->track_id));
  return BT::NodeStatus::SUCCESS;
}

// ---------------------------------------------------------------------------
// WaitForTargetSelected
// ---------------------------------------------------------------------------

WaitForTargetSelected::WaitForTargetSelected(
  const std::string & name, const BT::NodeConfig & config,
  const BT::RosNodeParams & params)
: BT::StatefulActionNode(name, config)
{
  auto node = params.nh.lock();
  if (!node) {
    throw BT::RuntimeError(
            "WaitForTargetSelected: RosNodeParams::nh expired");
  }
  logger_ = node->get_logger();
  cache_.init(node.get());
}

BT::PortsList WaitForTargetSelected::providedPorts()
{
  return {
    BT::InputPort<int>("timeout_ms", 60000, "Give up after this many ms"),
    BT::OutputPort<int64_t>("track_id", "Selected track id on SUCCESS"),
  };
}

BT::NodeStatus WaitForTargetSelected::onStart()
{
  getInput("timeout_ms", timeout_ms_);
  deadline_ = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(timeout_ms_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForTargetSelected::onRunning()
{
  yoloe_interfaces::msg::UnifiedTrackArray arr;
  if (cache_.snapshot(&arr)) {
    const auto * track = track_query::selected_track(arr);
    if (track != nullptr) {
      setOutput("track_id", static_cast<int64_t>(track->track_id));
      RCLCPP_INFO(
        logger_, "[%s] target selected: id=%lu class=%s",
        name().c_str(), static_cast<unsigned long>(track->track_id),
        track->class_name.c_str());
      return BT::NodeStatus::SUCCESS;
    }
  }
  if (std::chrono::steady_clock::now() >= deadline_) {
    RCLCPP_WARN(
      logger_, "[%s] no selected target within %d ms",
      name().c_str(), timeout_ms_);
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

// ---------------------------------------------------------------------------
// WaitUntilTargetNear
// ---------------------------------------------------------------------------

WaitUntilTargetNear::WaitUntilTargetNear(
  const std::string & name, const BT::NodeConfig & config,
  const BT::RosNodeParams & params)
: BT::StatefulActionNode(name, config)
{
  auto node = params.nh.lock();
  if (!node) {
    throw BT::RuntimeError("WaitUntilTargetNear: RosNodeParams::nh expired");
  }
  logger_ = node->get_logger();
  cache_.init(node.get());
}

BT::PortsList WaitUntilTargetNear::providedPorts()
{
  return {
    BT::InputPort<double>(
      "min_xy_m", 0.0, "Lower bound of the XoY arrival band (m)"),
    BT::InputPort<double>(
      "max_xy_m", 0.6, "Upper bound of the XoY arrival band (m)"),
    // 2026-11-19: 方位门限(rad)。目标相对臂基座 +X 轴的方位角必须
    // <= 该值才算"到达", 即只接受正前方 +-max_azimuth_rad 的扇区。
    // 0.2618 rad = 15 deg。<=0 关闭(旧行为)。
    BT::InputPort<double>(
      "max_azimuth_rad", 0.0,
      "Sector half-angle (rad); <=0 disables the azimuth criterion"),
    // 2026-11-20: 扇区中心方位角(rad)。0 = 臂基座 +X 轴; -1.5708 = -Y 轴。
    BT::InputPort<double>(
      "sector_center_rad", 0.0,
      "Sector centre azimuth in the arm base frame (rad); 0 = +X"),
    BT::InputPort<int>("timeout_ms", 120000, "Give up after this many ms"),
  };
}

BT::NodeStatus WaitUntilTargetNear::onStart()
{
  getInput("min_xy_m", min_xy_m_);
  getInput("max_xy_m", max_xy_m_);
  getInput("max_azimuth_rad", max_azimuth_rad_);
  getInput("sector_center_rad", sector_center_rad_);
  getInput("timeout_ms", timeout_ms_);
  deadline_ = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(timeout_ms_);
  RCLCPP_INFO(
    logger_,
    "[%s] waiting for target XoY in [%.2f, %.2f] m, azimuth within "
    "%.1f deg of %.1f deg",
    name().c_str(), min_xy_m_, max_xy_m_,
    max_azimuth_rad_ * 180.0 / 3.14159265358979323846,
    sector_center_rad_ * 180.0 / 3.14159265358979323846);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitUntilTargetNear::onRunning()
{
  yoloe_interfaces::msg::UnifiedTrackArray arr;
  if (cache_.snapshot(&arr)) {
    const auto * track = track_query::selected_track(arr);
    if (track == nullptr) {
      RCLCPP_WARN(
        logger_, "[%s] selected target disappeared (id=%lu)",
        name().c_str(), static_cast<unsigned long>(arr.selected_track_id));
      return BT::NodeStatus::FAILURE;
    }
    if (track_query::xy_in_band(*track, min_xy_m_, max_xy_m_) &&
      track_query::in_front_sector(*track, max_azimuth_rad_, sector_center_rad_))
    {
      // class + position included (2026-11-18): "which object did it go for"
      // was unanswerable from the logs — the tracker's per-tick trace is off
      // by default and YOLOE does not log per-frame classes.
      RCLCPP_INFO(
        logger_,
        "[%s] arrived: id=%lu class=%s XoY=%.3f m az=%.1f deg "
        "pos=(%.3f, %.3f, %.3f)",
        name().c_str(), static_cast<unsigned long>(track->track_id),
        track->class_name.c_str(), track_query::arm_base_xy_m(*track),
        track_query::arm_base_azimuth_rad(*track) * 180.0 / 3.14159265358979323846,
        track->position_arm_base.x, track->position_arm_base.y,
        track->position_arm_base.z);
      return BT::NodeStatus::SUCCESS;
    }
    // 1 Hz heartbeat: proves the tick loop is alive AND shows which track we
    // are actually waiting for (id / class / distance).  Its absence is the
    // fingerprint of a stalled tick thread.
    RCLCPP_INFO_THROTTLE(
      logger_, heartbeat_clock_, 1000,
      "[%s] still waiting: id=%lu class=%s XoY=%.3f m az=%.1f deg "
      "(band [%.2f, %.2f] m, az within +-%.1f deg of %.1f deg)",
      name().c_str(), static_cast<unsigned long>(track->track_id),
      track->class_name.c_str(), track_query::arm_base_xy_m(*track),
      track_query::arm_base_azimuth_rad(*track) * 180.0 / 3.14159265358979323846,
      min_xy_m_, max_xy_m_,
      max_azimuth_rad_ * 180.0 / 3.14159265358979323846,
      sector_center_rad_ * 180.0 / 3.14159265358979323846);
  } else {
    RCLCPP_INFO_THROTTLE(
      logger_, heartbeat_clock_, 1000,
      "[%s] still waiting: no track array yet", name().c_str());
  }
  if (std::chrono::steady_clock::now() >= deadline_) {
    RCLCPP_WARN(
      logger_, "[%s] target not within the arrival band after %d ms",
      name().c_str(), timeout_ms_);
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

// ---------------------------------------------------------------------------
// WaitForArmHits
// ---------------------------------------------------------------------------

WaitForArmHits::WaitForArmHits(
  const std::string & name, const BT::NodeConfig & config,
  const BT::RosNodeParams & params)
: BT::StatefulActionNode(name, config)
{
  auto node = params.nh.lock();
  if (!node) {
    throw BT::RuntimeError("WaitForArmHits: RosNodeParams::nh expired");
  }
  logger_ = node->get_logger();
  cache_.init(node.get());
}

BT::PortsList WaitForArmHits::providedPorts()
{
  return {
    BT::InputPort<int>("min_hits", 3, "Required arm-camera hits in the window"),
    BT::InputPort<int>("timeout_ms", 5000, "Give up after this many ms"),
  };
}

BT::NodeStatus WaitForArmHits::onStart()
{
  getInput("min_hits", min_hits_);
  getInput("timeout_ms", timeout_ms_);
  deadline_ = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(timeout_ms_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForArmHits::onRunning()
{
  yoloe_interfaces::msg::UnifiedTrackArray arr;
  if (cache_.snapshot(&arr)) {
    const auto * track = track_query::selected_track(arr);
    if (track != nullptr && track->arm_hits_in_window >= min_hits_) {
      RCLCPP_INFO(
        logger_, "[%s] target verified: id=%lu arm_hits=%d",
        name().c_str(), static_cast<unsigned long>(track->track_id),
        track->arm_hits_in_window);
      return BT::NodeStatus::SUCCESS;
    }
  }
  if (std::chrono::steady_clock::now() >= deadline_) {
    RCLCPP_WARN(
      logger_, "[%s] fewer than %d arm hits within %d ms",
      name().c_str(), min_hits_, timeout_ms_);
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

// ---------------------------------------------------------------------------
// IsArmCameraAlive
// ---------------------------------------------------------------------------

IsArmCameraAlive::IsArmCameraAlive(
  const std::string & name, const BT::NodeConfig & config,
  const BT::RosNodeParams & params)
: BT::SyncActionNode(name, config)
{
  auto node = params.nh.lock();
  if (!node) {
    throw BT::RuntimeError("IsArmCameraAlive: RosNodeParams::nh expired");
  }
  sub_ = node->create_subscription<yoloe_interfaces::msg::ObservationResult>(
    kArmResultTopic, rclcpp::QoS(10),
    [this](const yoloe_interfaces::msg::ObservationResult::SharedPtr msg) {
      on_result(msg);
    });
}

void IsArmCameraAlive::on_result(
  const yoloe_interfaces::msg::ObservationResult::SharedPtr /*msg*/)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_arrival_ = std::chrono::steady_clock::now();
  seen_ = true;
}

BT::PortsList IsArmCameraAlive::providedPorts()
{
  return {
    BT::InputPort<int>(
      "max_age_ms", 500, "Maximum age of the latest arm-camera message"),
  };
}

BT::NodeStatus IsArmCameraAlive::tick()
{
  getInput("max_age_ms", max_age_ms_);
  std::lock_guard<std::mutex> lock(mutex_);
  if (!seen_) {
    return BT::NodeStatus::FAILURE;
  }
  const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - last_arrival_).count();
  return age_ms <= max_age_ms_ ? BT::NodeStatus::SUCCESS
                               : BT::NodeStatus::FAILURE;
}

}  // namespace lgw_bt_planner
