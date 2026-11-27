// start_mapping_node.cpp
//
// Thin shell over point_lio's /start_mapping (std_srvs/Empty).  See the
// header for the one-shot/idempotent contract and the failure policy.

#include "lgw_bt_planner/start_mapping_node.hpp"

#include <rclcpp/logging.hpp>

namespace lgw_bt_planner
{

StartMappingNode::StartMappingNode(
  const std::string& name, const BT::NodeConfig& cfg,
  const BT::RosNodeParams& params)
: BT::RosServiceNode<Service>(name, cfg, params)
{
}

BT::PortsList StartMappingNode::providedPorts()
{
  // providedBasicPorts() exposes `service_name`, whose default is set in
  // bt_plugin_registration.cpp to "/start_mapping".  XML authors can
  // override per instance if a second LIO instance ever needs its own
  // trigger.
  return providedBasicPorts({});
}

bool StartMappingNode::setRequest(
  typename Service::Request::SharedPtr& /*req*/)
{
  return true;  // Empty request: nothing to fill in.
}

BT::NodeStatus StartMappingNode::onResponseReceived(
  const typename Service::Response::SharedPtr& /*resp*/)
{
  // The service latches mapping on inside point_lio.  Success here means
  // "the trigger was accepted", not "a map exists yet" -- downstream
  // nodes must still wait for /aft_pgo_odom before trusting the map.
  const auto node = node_.lock();
  RCLCPP_INFO(node ? node->get_logger() : rclcpp::get_logger("bt_runner"),
              "[StartMapping] /start_mapping accepted; point_lio is now "
              "mapping (wait for /aft_pgo_odom before relying on it)");
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus StartMappingNode::onFailure(BT::ServiceNodeErrorCode error)
{
  const auto node = node_.lock();
  RCLCPP_ERROR(node ? node->get_logger() : rclcpp::get_logger("bt_runner"),
               "[StartMapping] /start_mapping failed (%s) -- point_lio is "
               "still dropping lidar/IMU; check it is up and on the same "
               "ROS_DOMAIN_ID",
               BT::toStr(error));
  return BT::NodeStatus::FAILURE;
}

}  // namespace lgw_bt_planner
