#include "rclcpp/rclcpp.hpp"
#include "unitree_api/msg/request.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sport_model.hpp"
// 用本包自带的 nlohmann/json 3.11.2（include/nlohmann/），不是系统 apt 的
// 3.10.5：引号形式 + 本包的 -I 会优先命中本地副本。若要改成
// find_package(nlohmann_json) 走系统版本，先确认用到的 API 在 3.10.5 里齐全。
#include "nlohmann/json.hpp"

#include <algorithm>
#include <chrono>

using namespace std::placeholders;

// Go2 v0.2.0+ 步态 API（旧的 SwitchGait=1011 已删除）：
//   2049 ClassicWalk 是 flag 命令，参数 {"data": true} = "Ai 经典"/经典步态
//   2045 FreeWalk 是 trigger 命令（默认步态，无参数）
//
// 2026-11-05 真机结论：
//   BalanceStand(1002) / StopMove(1003) -> 固件丢弃经典步态 flag，回落 FreeWalk
//   Move(0,0,0)  (1008)                 -> 保持经典步态
//   纯空闲（什么都不发）                  -> 保持经典步态
//
// 所以本节点：
//   - 空闲一律发 **Move(0,0,0)**，绝不发 BalanceStand / StopMove；
//   - 步态只在「空闲 -> 运动」的那一帧补一次，**不做周期性重发**
//     （2026-11-20 用户要求：周期性重发会让狗隔一会儿就被"重新调一次步态"）。
class TwistBridge : public rclcpp::Node
{
public:
    TwistBridge() : Node("twist_bridge_node_cpp")
    {
        request_pub_ = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
        twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&TwistBridge::twist_cb, this, _1));

        keep_gait_ = this->declare_parameter<bool>("keep_gait", true);
        gait_api_id_ = this->declare_parameter<int>("gait_api_id", 2049);
        gait_is_flag_ = this->declare_parameter<bool>("gait_is_flag", true);
        gait_enable_ = this->declare_parameter<bool>("gait_enable", true);

        // 2026-11-20: 去掉周期性 gait 定时器。步态只在「空闲 -> 运动」的那一帧
        // 补一次即可；周期性重发会让狗隔一会儿就被"重新调一次步态"。
        RCLCPP_INFO(
            this->get_logger(),
            "TwistBridge创建: cmd_vel -> /api/sport/request; keep_gait=%s "
            "gait_api_id=%d gait_is_flag=%s enable=%s（空闲发 Move(0,0,0)，无周期重发）",
            keep_gait_ ? "true" : "false", gait_api_id_,
            gait_is_flag_ ? "true" : "false", gait_enable_ ? "true" : "false");
    }

private:
    void publish_gait()
    {
        unitree_api::msg::Request request;
        request.header.identity.api_id = gait_api_id_;
        if (gait_is_flag_) {
            nlohmann::json js;
            js["data"] = gait_enable_;
            request.parameter = js.dump();
        }
        request_pub_->publish(request);
    }

    void twist_cb(const geometry_msgs::msg::Twist::SharedPtr twist)
    {
        unitree_api::msg::Request request;

        const double x = twist->linear.x;
        const double y = twist->linear.y;
        const double z = twist->angular.z;
        const bool moving = (x != 0.0 || y != 0.0 || z != 0.0);

        if (moving) {
            // 空闲 -> 运动的第一帧，先补一次步态，避免第一段路用默认步态跑。
            if (!was_moving_ && keep_gait_) {
                publish_gait();
            }
            request.header.identity.api_id = ROBOT_SPORT_API_ID_MOVE;
            nlohmann::json js;
            js["x"] = x;
            js["y"] = y;
            js["z"] = z;
            request.parameter = js.dump();
        } else {
            // 空闲用 Move(0,0,0)：**不会**让固件丢弃经典步态。
            // BalanceStand(1002) / StopMove(1003) 都会，所以一律不用。
            request.header.identity.api_id = ROBOT_SPORT_API_ID_MOVE;
            nlohmann::json js;
            js["x"] = 0.0;
            js["y"] = 0.0;
            js["z"] = 0.0;
            request.parameter = js.dump();
        }

        request_pub_->publish(request);
        was_moving_ = moving;
    }

    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr request_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;

    bool keep_gait_{true};
    int gait_api_id_{2049};
    bool gait_is_flag_{true};
    bool gait_enable_{true};
    bool was_moving_{false};
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TwistBridge>());
    rclcpp::shutdown();
    return 0;
}
