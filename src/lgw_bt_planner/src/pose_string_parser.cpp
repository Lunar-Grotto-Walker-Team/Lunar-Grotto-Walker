#include "lgw_bt_planner/pose_string_parser.hpp"

#include <sstream>
#include <stdexcept>
#include <vector>

namespace
{

// Split a string by ';' into trimmed tokens. Empty trailing token kept.
std::vector<std::string> split(const std::string& s, char delim)
{
  std::vector<std::string> out;
  std::string cur;
  std::istringstream iss(s);
  while (std::getline(iss, cur, delim)) {
    // trim ASCII whitespace
    size_t a = cur.find_first_not_of(" \t\r\n");
    size_t b = cur.find_last_not_of(" \t\r\n");
    out.push_back(a == std::string::npos ? std::string() : cur.substr(a, b - a + 1));
  }
  return out;
}

}  // namespace

namespace lgw_bt_planner
{

geometry_msgs::msg::PoseStamped parse_pose_string(BT::StringView str)
{
  geometry_msgs::msg::PoseStamped out;
  out.header.frame_id = "base_link";   // sensible default
  out.pose.orientation.w = 1.0;        // identity quaternion

  const std::string s(str.data(), str.size());
  const auto tokens = split(s, ';');
  if (tokens.size() < 1 || tokens[0].empty()) {
    throw std::runtime_error(
      "PoseStamped string must start with a non-empty frame_id; got: '" + s + "'");
  }
  out.header.frame_id = tokens[0];

  auto parse_double = [](const std::string& tok, double fallback) -> double {
    if (tok.empty()) return fallback;
    try {
      return std::stod(tok);
    } catch (const std::exception&) {
      throw std::runtime_error("Invalid float in PoseStamped literal: '" + tok + "'");
    }
  };

  // tokens: [frame, x, y, z, qx, qy, qz, qw]
  if (tokens.size() > 1) out.pose.position.x = parse_double(tokens[1], 0.0);
  if (tokens.size() > 2) out.pose.position.y = parse_double(tokens[2], 0.0);
  if (tokens.size() > 3) out.pose.position.z = parse_double(tokens[3], 0.0);
  if (tokens.size() > 4) out.pose.orientation.x = parse_double(tokens[4], 0.0);
  if (tokens.size() > 5) out.pose.orientation.y = parse_double(tokens[5], 0.0);
  if (tokens.size() > 6) out.pose.orientation.z = parse_double(tokens[6], 0.0);
  if (tokens.size() > 7) out.pose.orientation.w = parse_double(tokens[7], 1.0);

  if (tokens.size() > 8) {
    throw std::runtime_error(
      "PoseStamped literal has more than 8 ';'-separated fields: '" + s + "'");
  }
  return out;
}

}  // namespace lgw_bt_planner

namespace BT
{

template <>
inline geometry_msgs::msg::PoseStamped
convertFromString<geometry_msgs::msg::PoseStamped>(const StringView str)
{
  return lgw_bt_planner::parse_pose_string(str);
}

}  // namespace BT
