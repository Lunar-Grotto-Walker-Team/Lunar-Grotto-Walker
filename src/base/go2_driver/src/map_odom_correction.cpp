// map_odom_correction.cpp
//
// 动态 map -> odom 修正（2026-10-23，2026-10-30 改用 PGO 位姿）
//
// 背景
// ----
// go2_nav2_slam_explore.launch.py 原来发一条"单位变换"的静态 map -> odom。
// 它只有在下面两个前提同时成立时才对：
//   1) Terminal A(FAST-LIO + elevation_mapping) 和 Terminal B(nav2) 同时启动；
//   2) 狗在这期间没挪窝。
// 因为 map 的原点是 SLAM 启动瞬间 body 的位置，而 odom 的原点是 Go2
// 内部里程计(/utlidar/robot_pose)的参考原点 —— 两者完全无关。
// 实机实测：狗站着不动，/utlidar/robot_pose 已经是 (-26.90, -18.87)，于是
// nav2 把狗摆到高程图 (0,0) 之外 33m，global_costmap 刷
//   "Robot is out of bounds of the costmap!"
//   "Sensor origin at (-27.21, -18.92) is out of map bounds (-25.00, -25.00) ..."
//
// 本节点做的事
// ------------
// 以 20Hz 发布动态 map -> odom，使得
//
//   T_map_odom = T_map_body * (T_base_body)^-1 * (T_odom_base)^-1
//
// 其中 T_base_body 是 **body->base_footprint** 的静态安装外参（base <- body，
// 平移为 body 原点在 base 系中的位置），由 map_odom_math.hpp 内部求逆。
// 与 relay(pointlio_cloud_relay/transform.py) 的外参语义保持一致。
//
// T_map_body 的来源（二选一，prefer_pgo_odom 控制）：
//   1) /aft_pgo_odom (nav_msgs/Odometry)：point_lio 回环校正后的 body 位姿，
//      其父坐标系为 camera_init；map -> camera_init 是 identity 静态 TF，
//      故可直接当作 T_map_body 使用（推荐，默认）；
//   2) 回退：查 TF map_frame <- body_frame，即原始 LIO 位姿。
//
// PGO 位姿带超时：只有最新一条到达后 pgo_timeout_sec（默认 3s）内才使用；
// 超时视为 SLC 停发，自动回退到 TF，避免一直广播冻结的 map -> odom。
//
// 2026-11-06 源切换迟滞
// --------------------
// SLC 在回环优化期间 /aft_pgo_odom 会断流 1-2 s。原实现（timeout=1s）会在
// "PGO 位姿" 与 "原始 LIO TF" 之间反复切换；两者相差回环累计校正量，每切
// 一次 map->odom 就跳一次，global_costmap 会累积出多条不同方向的走廊。
// 现在的策略：
//   - pgo_timeout_sec 内继续用最后一条 PGO 位姿（短暂冻结，不切换）；
//   - 超过 pgo_timeout_sec 才回退到原始 LIO TF；
//   - 回退后要求 PGO 连续新鲜 pgo_recover_sec 才切回（避免边界 ping-pong）；
//   - 每次真正的源切换打 INFO（带 map->odom 跳变 Δ），跳变超阈值时调用
//     /global_costmap/clear_entirely_global_costmap 清掉旧对齐残留。
//
// 于是 TF 链
//   map -> odom -> base_footprint      (nav2 全局位姿)
// 恒等于
//   map -> camera_init -> body -> base_footprint   (与 elevation_mapping 同源)
//
// 效果：
//   - global_costmap / terrain_traversability_layer / planner 用的全局位姿
//     与建图位姿完全一致，不再受狗内部里程计初值/漂移影响；
//   - local_costmap 仍用 odom 系(driver 发的 odom -> base_footprint)，保持
//     低延迟、平滑，两棵树依旧连通；
//   - map -> odom 会随 SLAM/PGO 位姿连续变化，狗的全局位姿不再靠"起点重合"。
//
// 使用前提
// --------
//   - 回退路径需要 Terminal A 在跑（否则 map -> body 取不到，本节点不发布，
//     并打 WARN）；
//   - 本节点运行时，go2_nav2_slam_explore.launch.py 必须
//       publish_map_to_odom:=false
//     否则同一条 map -> odom 有两个广播者，TF 会打架。launch 里已做互斥。
//
// body -> base_footprint 的静态安装外参默认值来自安装链标定，见
// map_odom_math.hpp / transform.py；标定/换安装位时可改参数。

#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <Eigen/Geometry>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "go2_driver/map_odom_math.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace go2_driver
{

namespace
{

tf2::Transform toTf(const geometry_msgs::msg::Transform & msg)
{
  tf2::Transform out;
  out.setOrigin(
    tf2::Vector3(
      msg.translation.x, msg.translation.y, msg.translation.z));
  out.setRotation(
    tf2::Quaternion(
      msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w));
  return out;
}

geometry_msgs::msg::Transform toMsg(const tf2::Transform & tf)
{
  geometry_msgs::msg::Transform out;
  const tf2::Vector3 & o = tf.getOrigin();
  const tf2::Quaternion q = tf.getRotation();
  out.translation.x = o.x();
  out.translation.y = o.y();
  out.translation.z = o.z();
  out.rotation.x = q.x();
  out.rotation.y = q.y();
  out.rotation.z = q.z();
  out.rotation.w = q.w();
  return out;
}

Eigen::Matrix4d toEigen(const tf2::Transform & tf)
{
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  const tf2::Vector3 & o = tf.getOrigin();
  const tf2::Quaternion q = tf.getRotation();
  const Eigen::Quaterniond eq(q.w(), q.x(), q.y(), q.z());
  T.block<3, 3>(0, 0) = eq.normalized().toRotationMatrix();
  T(0, 3) = o.x();
  T(1, 3) = o.y();
  T(2, 3) = o.z();
  return T;
}

Eigen::Matrix4d toEigen(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  const Eigen::Quaterniond q(
    pose.orientation.w, pose.orientation.x,
    pose.orientation.y, pose.orientation.z);
  T.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
  T(0, 3) = pose.position.x;
  T(1, 3) = pose.position.y;
  T(2, 3) = pose.position.z;
  return T;
}

tf2::Transform toTf(const Eigen::Matrix4d & T)
{
  const Eigen::Quaterniond q(Eigen::Matrix3d(T.block<3, 3>(0, 0)));
  const Eigen::Quaterniond qn = q.normalized();
  tf2::Transform out(
    tf2::Quaternion(qn.x(), qn.y(), qn.z(), qn.w()),
    tf2::Vector3(T(0, 3), T(1, 3), T(2, 3)));
  return out;
}

}  // namespace

class MapOdomCorrection : public rclcpp::Node
{
public:
  MapOdomCorrection()
  : rclcpp::Node("map_odom_correction")
  {
    map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
    body_frame_ = this->declare_parameter<std::string>("body_frame", "aft_mapped");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_footprint");
    publish_rate_ = this->declare_parameter<double>("publish_rate", 20.0);

    // PGO 位姿来源
    pgo_odom_topic_ = this->declare_parameter<std::string>(
      "pgo_odom_topic", "/aft_pgo_odom");
    prefer_pgo_odom_ = this->declare_parameter<bool>("prefer_pgo_odom", true);
    pgo_timeout_sec_ = this->declare_parameter<double>("pgo_timeout_sec", 1.0);
    // 2026-11-06: 源切换迟滞。见文件头注释。
    pgo_recover_sec_ = this->declare_parameter<double>("pgo_recover_sec", 2.0);
    clear_costmap_on_switch_ =
      this->declare_parameter<bool>("clear_costmap_on_switch", true);
    switch_clear_translation_m_ =
      this->declare_parameter<double>("switch_clear_translation_m", 0.15);
    switch_clear_yaw_rad_ =
      this->declare_parameter<double>("switch_clear_yaw_rad", 0.10);

    // body -> base_footprint 静态安装外参，语义 T^base_body（base <- body）：
    // 平移 = body 原点在 base 系中的位置，与 relay 一致。
    body_to_base_x_ = this->declare_parameter<double>("body_to_base_x", 0.187793);
    body_to_base_y_ = this->declare_parameter<double>("body_to_base_y", 0.02329);
    body_to_base_z_ = this->declare_parameter<double>("body_to_base_z", 0.391836);
    body_to_base_roll_ = this->declare_parameter<double>("body_to_base_roll", 0.0);
    body_to_base_pitch_ = this->declare_parameter<double>("body_to_base_pitch", 0.2269);
    body_to_base_yaw_ = this->declare_parameter<double>("body_to_base_yaw", 0.0);

    if (publish_rate_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "publish_rate<=0, fallback to 20Hz");
      publish_rate_ = 20.0;
    }
    if (pgo_timeout_sec_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "pgo_timeout_sec<=0, fallback to 1.0s");
      pgo_timeout_sec_ = 1.0;
    }

    // 固定外参矩阵：T_base_body（base <- body）
    {
      const Eigen::Quaterniond q =
        Eigen::AngleAxisd(body_to_base_yaw_, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(body_to_base_pitch_, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(body_to_base_roll_, Eigen::Vector3d::UnitX());
      T_base_body_ = Eigen::Matrix4d::Identity();
      T_base_body_.block<3, 3>(0, 0) = q.toRotationMatrix();
      T_base_body_(0, 3) = body_to_base_x_;
      T_base_body_(1, 3) = body_to_base_y_;
      T_base_body_(2, 3) = body_to_base_z_;
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    pgo_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      pgo_odom_topic_, rclcpp::QoS(10),
      std::bind(&MapOdomCorrection::pgoCallback, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = this->create_wall_timer(period, std::bind(&MapOdomCorrection::timerCallback, this));

    if (clear_costmap_on_switch_) {
      // 源切换且 map->odom 跳变超阈值时清 global costmap，避免旧对齐的
      // 地形/障碍标记残留成"多条走廊"。服务不在时只告警，不影响发布。
      clear_costmap_client_ = this->create_client<std_srvs::srv::Empty>(
        "/global_costmap/clear_entirely_global_costmap");
    }

    // 2026-11-06: 11-05 真机出现过 map->odom 停止更新 ~41 s、而本节点日志
    // 一条 WARN 都没有的情况。单线程 executor 里 timerCallback 一旦阻塞
    // (最可疑是 tf_broadcaster_->sendTransform 的 DDS 可靠发布), 就没有任何
    // 回调能报告故障。这里用独立线程做看门狗, 只读两个 steady_clock 时间戳,
    // 不碰 tf/DDS, 因此主线程被阻塞时它仍能打日志。
    last_publish_steady_ns_.store(steadyNowNs());
    watchdog_ = std::thread([this]() { this->watchdogLoop(); });

    RCLCPP_INFO(
      this->get_logger(),
      "map_odom_correction started: map=%s odom=%s body=%s base=%s rate=%.1fHz "
      "pgo_topic=%s prefer_pgo=%s pgo_timeout=%.1fs pgo_recover=%.1fs "
      "clear_on_switch=%s (%.2fm/%.2frad) "
      "body->base=(%.5f, %.5f, %.5f) rpy=(%.4f, %.4f, %.4f)",
      map_frame_.c_str(), odom_frame_.c_str(), body_frame_.c_str(), base_frame_.c_str(),
      publish_rate_, pgo_odom_topic_.c_str(), prefer_pgo_odom_ ? "true" : "false",
      pgo_timeout_sec_, pgo_recover_sec_,
      clear_costmap_on_switch_ ? "true" : "false",
      switch_clear_translation_m_, switch_clear_yaw_rad_,
      body_to_base_x_, body_to_base_y_, body_to_base_z_,
      body_to_base_roll_, body_to_base_pitch_, body_to_base_yaw_);
  }

  ~MapOdomCorrection() override
  {
    stop_watchdog_.store(true);
    if (watchdog_.joinable()) {
      watchdog_.join();
    }
  }

private:
  enum class PoseSource
  {
    NONE,
    PGO_ODOM,
    TF,
  };

  static int64_t steadyNowNs()
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  static int sourceCode(PoseSource source)
  {
    return source == PoseSource::PGO_ODOM ? 1 :
           (source == PoseSource::TF ? 2 : 0);
  }

  static const char * sourceName(int code)
  {
    switch (code) {
      case 1: return "PGO_ODOM";
      case 2: return "TF";
      default: return "NONE";
    }
  }

  // 独立线程: 只读两个 atomic 时间戳, 不碰 tf/DDS, 主线程被阻塞时仍能输出。
  void watchdogLoop()
  {
    constexpr double kStaleSec = 1.0;
    bool stale = false;
    int64_t stale_since_ns = 0;
    while (!stop_watchdog_.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      const int64_t last_pub = last_publish_steady_ns_.load();
      const int64_t last_enter = timer_enter_steady_ns_.load();
      const int64_t last_exit = timer_exit_steady_ns_.load();
      const int64_t now = steadyNowNs();
      const double age = (now - last_pub) * 1e-9;
      const double enter_age = last_enter == 0 ? age : (now - last_enter) * 1e-9;
      if (age > kStaleSec) {
        if (!stale) {
          stale = true;
          stale_since_ns = last_pub;
          const char * reason;
          if (last_enter == 0 || enter_age > kStaleSec) {
            reason = "timer 回调未运行（executor 被其它回调阻塞，或定时器停止）";
          } else if (last_exit < last_enter) {
            reason = "卡在 timerCallback 内部（最可能是 /tf 的 sendTransform 阻塞）";
          } else {
            reason = "回调已正常返回但未发布（上方应有 PGO/TF 相关 WARN）";
          }
          RCLCPP_WARN(
            this->get_logger(),
            "map->odom 已 %.1f s 未发布（上次来源=%s）: %s",
            age, sourceName(last_source_code_.load()), reason);
        }
      } else if (stale) {
        stale = false;
        RCLCPP_INFO(
          this->get_logger(), "map->odom 恢复发布（中断 %.1f s）",
          (now - stale_since_ns) * 1e-9);
      }
    }
  }

  void pgoCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(pgo_mutex_);
    pgo_odom_ = *msg;
    // Node-clock arrival time, not the message stamp (lidar/device clock).
    last_pgo_time_ = this->now();
    have_pgo_ = true;
  }

  void timerCallback()
  {
    timer_enter_steady_ns_.store(steadyNowNs());
    // 无论从哪个 return 退出，都记录退出时刻，供看门狗区分
    // "没进回调 / 卡在回调里 / 回调返回但没发布"。
    struct ExitStamp
    {
      std::atomic<int64_t> * slot;
      ~ExitStamp()
      {
        slot->store(std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count());
      }
    } exit_stamp{&timer_exit_steady_ns_};
    Eigen::Matrix4d T_map_body;
    PoseSource source = PoseSource::NONE;

    if (prefer_pgo_odom_) {
      std::lock_guard<std::mutex> lock(pgo_mutex_);
      const double age_sec = have_pgo_ ?
        (this->now() - last_pgo_time_).seconds() :
        std::numeric_limits<double>::infinity();
      const bool pgo_available = have_pgo_ && isPgoFresh(age_sec, pgo_timeout_sec_);

      bool use_pgo = false;
      if (pgo_available) {
        if (active_source_ == PoseSource::TF) {
          // 2026-11-06: 回退后不要一有新消息就切回，要求连续新鲜
          // pgo_recover_sec_，避免在超时边界上 PGO<->TF 反复横跳。
          pgo_fresh_streak_sec_ += 1.0 / publish_rate_;
          use_pgo = pgo_fresh_streak_sec_ >= pgo_recover_sec_;
        } else {
          // NONE（启动）或已经在 PGO 上：保持。
          use_pgo = true;
          pgo_fresh_streak_sec_ = 0.0;
        }
      } else {
        pgo_fresh_streak_sec_ = 0.0;
      }

      if (use_pgo) {
        // /aft_pgo_odom 的 pose 是 camera_init 系下的 body 位姿；
        // map -> camera_init 为 identity 静态 TF，故可直接作为 T_map_body。
        // 注意：pgo_timeout_sec_ 内即使最新一条略旧也继续用它（保持最后
        // PGO 位姿），这是避免源切换跳变的关键；超过才回退到原始 LIO TF。
        T_map_body = toEigen(pgo_odom_.pose.pose);
        source = PoseSource::PGO_ODOM;
      } else if (age_sec > pgo_timeout_sec_) {
        // 只有真正超过保持窗口才告警；回退后的恢复期不刷日志。
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "no %s within %.1f s, falling back to TF %s<-%s",
          pgo_odom_topic_.c_str(), pgo_timeout_sec_,
          map_frame_.c_str(), body_frame_.c_str());
      }
    }

    if (source == PoseSource::NONE) {
      try {
        // TimePointZero: 取 buffer 里最新一帧，不阻塞等待。
        // SLAM 的 TF 时间戳来自雷达/设备时钟，与本机时钟有固定偏差，
        // 用 now() 去查会一直 extrapolation failure。
        const auto map_body = tf_buffer_->lookupTransform(
          map_frame_, body_frame_, tf2::TimePointZero);
        T_map_body = toEigen(toTf(map_body.transform));
        source = PoseSource::TF;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 3000,
          "map->odom 暂不可用（/aft_pgo_odom 未到达，且 TF %s<-%s 查不到；"
          "SLAM/driver 是否在跑？）: %s",
          map_frame_.c_str(), body_frame_.c_str(), ex.what());
        return;
      }
    }

    Eigen::Matrix4d T_odom_base;
    try {
      const auto odom_base = tf_buffer_->lookupTransform(
        odom_frame_, base_frame_, tf2::TimePointZero);
      T_odom_base = toEigen(toTf(odom_base.transform));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "map->odom 暂不可用（driver 是否在发 odom->%s？）: %s",
        base_frame_.c_str(), ex.what());
      return;
    }

    const Eigen::Matrix4d T_map_odom =
      computeMapOdom(T_map_body, T_base_body_, T_odom_base);

    geometry_msgs::msg::TransformStamped out;
    out.header.stamp = this->now();
    out.header.frame_id = map_frame_;
    out.child_frame_id = odom_frame_;
    out.transform = toMsg(toTf(T_map_odom));
    tf_broadcaster_->sendTransform(out);
    last_publish_steady_ns_.store(steadyNowNs());

    handleSourceSwitch(source, T_map_odom);
  }

  void handleSourceSwitch(PoseSource source, const Eigen::Matrix4d & T_map_odom)
  {
    last_source_code_.store(
      source == PoseSource::PGO_ODOM ? 1 :
      (source == PoseSource::TF ? 2 : 0));

    if (active_source_ == PoseSource::NONE) {
      active_source_ = source;
      last_T_map_odom_ = T_map_odom;
      has_last_T_map_odom_ = true;
      RCLCPP_INFO(
        this->get_logger(), "map->odom 初始源: %s", sourceName(sourceCode(source)));
      return;
    }

    if (source == active_source_) {
      last_T_map_odom_ = T_map_odom;
      has_last_T_map_odom_ = true;
      return;
    }

    // 真正的源切换：先量化 map->odom 的跳变，必要时清 global costmap。
    double dt = 0.0;
    double dyaw = 0.0;
    if (has_last_T_map_odom_) {
      dt = (T_map_odom.block<3, 1>(0, 3) - last_T_map_odom_.block<3, 1>(0, 3)).norm();
      const Eigen::Matrix3d dR =
        T_map_odom.block<3, 3>(0, 0) * last_T_map_odom_.block<3, 3>(0, 0).transpose();
      dyaw = std::abs(std::atan2(dR(1, 0), dR(0, 0)));
    }
    RCLCPP_INFO(
      this->get_logger(),
      "map->odom 源切换 %s -> %s（Δ=%.3f m, %.1f deg）",
      sourceName(sourceCode(active_source_)), sourceName(sourceCode(source)),
      dt, dyaw * 180.0 / M_PI);

    if (clear_costmap_on_switch_ && has_last_T_map_odom_ &&
      (dt > switch_clear_translation_m_ || dyaw > switch_clear_yaw_rad_))
    {
      clearGlobalCostmap();
    }

    active_source_ = source;
    last_T_map_odom_ = T_map_odom;
    has_last_T_map_odom_ = true;
  }

  void clearGlobalCostmap()
  {
    if (!clear_costmap_client_) {
      return;
    }
    if (!clear_costmap_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "源切换需要清 global costmap，但 "
        "/global_costmap/clear_entirely_global_costmap 未就绪");
      return;
    }
    clear_costmap_client_->async_send_request(
      std::make_shared<std_srvs::srv::Empty::Request>(),
      [this](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture) {
        RCLCPP_INFO(this->get_logger(), "源切换后已清空 global costmap");
      });
  }

  // frames
  std::string map_frame_;
  std::string odom_frame_;
  std::string body_frame_;
  std::string base_frame_;

  // params
  double publish_rate_;
  bool prefer_pgo_odom_;
  std::string pgo_odom_topic_;
  double pgo_timeout_sec_;
  double pgo_recover_sec_{2.0};
  bool clear_costmap_on_switch_{true};
  double switch_clear_translation_m_{0.15};
  double switch_clear_yaw_rad_{0.10};
  double body_to_base_x_;
  double body_to_base_y_;
  double body_to_base_z_;
  double body_to_base_roll_;
  double body_to_base_pitch_;
  double body_to_base_yaw_;
  Eigen::Matrix4d T_base_body_ = Eigen::Matrix4d::Identity();

  // pgo odom input
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pgo_sub_;
  std::mutex pgo_mutex_;
  nav_msgs::msg::Odometry pgo_odom_;
  bool have_pgo_ = false;
  rclcpp::Time last_pgo_time_;

  // tf
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

  // 源切换迟滞状态
  PoseSource active_source_ = PoseSource::NONE;
  double pgo_fresh_streak_sec_{0.0};
  Eigen::Matrix4d last_T_map_odom_ = Eigen::Matrix4d::Identity();
  bool has_last_T_map_odom_{false};
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_costmap_client_;

  // 看门狗状态（独立线程只读，timer 回调只写）
  std::atomic<int64_t> last_publish_steady_ns_{0};
  std::atomic<int64_t> timer_enter_steady_ns_{0};
  std::atomic<int64_t> timer_exit_steady_ns_{0};
  std::atomic<int> last_source_code_{0};
  std::atomic<bool> stop_watchdog_{false};
  std::thread watchdog_;
};

}  // namespace go2_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<go2_driver::MapOdomCorrection>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
