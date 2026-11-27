#pragma once

#include <behaviortree_cpp/basic_types.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <string>
#include <string_view>

namespace BT
{

// BT.ROS2 / SetBlackboard cannot natively parse geometry_msgs::msg::PoseStamped
// from a string. The demo XML uses the compact format:
//   "frame_id;x;y;z;qx;qy;qz;qw"
// Splitting on ';' gives 8 fields; missing fields default to identity frame +
// zero translation + identity orientation (qw=1).
template <>
inline geometry_msgs::msg::PoseStamped
convertFromString<geometry_msgs::msg::PoseStamped>(const StringView str);

}  // namespace BT

namespace lgw_bt_planner
{

// Standalone helper for non-BT contexts (e.g. tests or direct API calls).
// Takes BT::StringView (== std::string_view in BT.CPP v4) to share a single
// conversion path with convertFromString<PoseStamped>.
geometry_msgs::msg::PoseStamped parse_pose_string(BT::StringView str);

}  // namespace lgw_bt_planner