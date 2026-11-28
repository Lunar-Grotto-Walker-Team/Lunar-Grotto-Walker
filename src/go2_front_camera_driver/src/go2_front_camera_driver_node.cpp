// go2_front_camera_driver_node.cpp
//
// Unitree Go2 front monocular camera driver.
// Pulls H.264 RTP over UDP multicast 230.1.1.1:1720 via GStreamer,
// publishes sensor_msgs/Image + sensor_msgs/CameraInfo pair on
// realsense-aligned topics.
//
// Design references:
//   - l4t/autonomy_stack_go2/.../go2_h264_repub.cpp (GStreamer pipeline string)
//   - backup/go2_ros2_sdk/.../publish_camera_data.py (Image+CameraInfo pair publish)
//   - backup/go2_ros2_sdk/.../camera_config.py       (CameraInfo YAML loader pattern)

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "camera_info_manager/camera_info_manager.hpp"
#include "opencv2/opencv.hpp"

using std::chrono::milliseconds;

class Go2FrontCameraDriver : public rclcpp::Node {
public:
  Go2FrontCameraDriver() : Node("go2_front_camera_driver") {
    // ---- Parameters (all overridable from launch) ----
    declare_parameter("multicast_iface",      std::string("eth0"));
    declare_parameter("image_topic",          std::string("/camera/frontcamera/image_raw"));
    declare_parameter("camera_info_topic",    std::string("/camera/frontcamera/camera_info"));
    declare_parameter("frame_id",             std::string("go2_front_camera_optical_frame"));
    declare_parameter("camera_info_url",
      std::string("package://go2_front_camera_driver/config/front_camera_720.yaml"));

    const auto image_topic       = get_parameter("image_topic").as_string();
    const auto camera_info_topic = get_parameter("camera_info_topic").as_string();
    const auto camera_info_url   = get_parameter("camera_info_url").as_string();

    // ---- Publishers (BEST_EFFORT + depth=1, realsense SENSOR_DATA style) ----
    auto sensor_qos = rclcpp::SensorDataQoS();
    image_pub_ = create_publisher<sensor_msgs::msg::Image>(image_topic, sensor_qos);
    info_pub_  = create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic, sensor_qos);

    // ---- CameraInfo loader (camera_info_manager; replaces backup's Python loader) ----
    camera_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(
      this, "go2_front_camera");
    if (!camera_info_manager_->loadCameraInfo(camera_info_url)) {
      RCLCPP_WARN(get_logger(),
        "Failed to load camera_info from %s, will publish empty CameraInfo. "
        "Run camera_calibration to generate a valid yaml.",
        camera_info_url.c_str());
    }

    // ---- GStreamer pipeline (extended from go2_h264_repub.cpp:15) ----
    // Go2 streams H.264 RTP/UDP multicast 230.1.1.1:1720. Real frame rate is
    // 14.5 fps (not 15 / 30 — verified with gst-launch probe). The original
    // minimal pipeline silently scrambled under mild jitter because:
    //   - default udpsrc socket buffer (~200 KB) overflows on h264 frames,
    //   - rtph264depay needs clock-rate=90000 to reassemble NALUs,
    //   - no rtpjitterbuffer means out-of-order RTP packets corrupt P-frames.
    // Tested: with the additions below, fpsdisplaysink reports 0 dropped at
    // stable 14.5 fps over 8 s.
    const auto iface = get_parameter("multicast_iface").as_string();
    const std::string pipeline =
      "udpsrc address=230.1.1.1 port=1720 multicast-iface=" + iface +
      " buffer-size=8388608"
      " ! application/x-rtp, media=video, encoding-name=H264, clock-rate=90000"
      " ! rtpjitterbuffer latency=50"
      " ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert"
      " ! video/x-raw,width=1280,height=720,format=BGR ! appsink drop=1 max-buffers=1";

    cap_.open(pipeline, cv::CAP_GSTREAMER);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(get_logger(),
        "Failed to open GStreamer pipeline (multicast_iface=%s). "
        "Check that the Go2 is on and that you are on the same subnet (eth0/eth10).",
        iface.c_str());
      throw std::runtime_error("GStreamer pipeline open failed");
    }

    // ---- 14.3 Hz timer (Go2 onboard stream is 14.5 fps; 70 ms = 14.3 Hz
    // slightly under-samples so appsink always has a fresh frame ready and
    // we never pull faster than the producer — avoids unnecessary read() churn).
    timer_ = create_wall_timer(milliseconds(70),
      std::bind(&Go2FrontCameraDriver::on_timer, this));

    RCLCPP_INFO(get_logger(),
      "Go2 front camera driver started. image=%s info=%s frame_id=%s iface=%s",
      image_topic.c_str(), camera_info_topic.c_str(),
      get_parameter("frame_id").as_string().c_str(), iface.c_str());
  }

private:
  void on_timer() {
    cv::Mat frame;
    if (!cap_.read(frame) || frame.empty()) {
      return;
    }

    // Compute stamp ONCE; share across image and camera_info so pair subscribers
    // (e.g. message_filters) can match exactly.
    const auto stamp    = now();
    const auto frame_id = get_parameter("frame_id").as_string();

    auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
    img_msg->header.stamp    = stamp;
    img_msg->header.frame_id = frame_id;
    image_pub_->publish(*img_msg);

    auto info_msg = camera_info_manager_->getCameraInfo();
    info_msg.header = img_msg->header;
    info_pub_->publish(info_msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr    image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  std::unique_ptr<camera_info_manager::CameraInfoManager>   camera_info_manager_;
  cv::VideoCapture                                          cap_;
  rclcpp::TimerBase::SharedPtr                              timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go2FrontCameraDriver>());
  rclcpp::shutdown();
  return 0;
}
