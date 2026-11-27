// start_mapping_node.hpp
//
// BT thin shell over point_lio's /start_mapping (std_srvs/Empty).
//
// Why it exists: Point-LIO can come up warm (drivers, DDS, filters, map
// allocated) while *not* consuming lidar/IMU, so the map origin is wherever
// the operator finally places the robot rather than wherever it happened to
// be during warm-up.  With point_lio's `wait_for_start:=true`, the first
// /start_mapping call latches mapping on; the service is one-shot and
// idempotent (a second call is a no-op), which is why this node returns
// SUCCESS and does not need a Retry decorator to "keep trying".
//
// Failure semantics: if point_lio is not up yet the service call fails
// (SERVICE_UNREACHABLE / SERVICE_TIMEOUT).  That must NOT be swallowed —
// a dog that believes it is mapping while LIO is still dropping samples is
// exactly the failure this node is meant to prevent.  Map it to FAILURE and
// let the tree author decide (RetryUntilSuccessful, fallback branch, or an
// operator-visible log).
//
// Domain note: bt_runner and point_lio must share ROS_DOMAIN_ID (0 on the
// real robot).  Cross-domain gives SERVICE_UNREACHABLE forever.

#pragma once

#include <behaviortree_ros2/bt_service_node.hpp>

#include <std_srvs/srv/empty.hpp>

#include <string>

namespace lgw_bt_planner
{

class StartMappingNode : public BT::RosServiceNode<std_srvs::srv::Empty>
{
public:
  using Service = std_srvs::srv::Empty;

  StartMappingNode(const std::string& name, const BT::NodeConfig& config,
                   const BT::RosNodeParams& params);

  StartMappingNode() = delete;
  StartMappingNode(const StartMappingNode&) = delete;
  StartMappingNode& operator=(const StartMappingNode&) = delete;

  static BT::PortsList providedPorts();

protected:
  bool setRequest(typename Service::Request::SharedPtr& req) override;

  BT::NodeStatus onResponseReceived(
    const typename Service::Response::SharedPtr& resp) override;

  BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;
};

}  // namespace lgw_bt_planner
