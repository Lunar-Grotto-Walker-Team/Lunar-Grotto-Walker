// traversability_layer.hpp
#ifndef NAV2_TRAVERSABILITY_PLUGIN__TRAVERSABILITY_LAYER_HPP_
#define NAV2_TRAVERSABILITY_PLUGIN__TRAVERSABILITY_LAYER_HPP_

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <mutex>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>

#include "nav2_traversability_plugin/terrain_cost.hpp"
#include "nav2_traversability_plugin/terrain_grid_alignment.hpp"

namespace nav2_traversability_plugin
{
class TraversabilityLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  TraversabilityLayer();
  virtual void onInitialize() override;
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y) override;
  virtual void updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) override;
  virtual void onFootprintChanged() override;

  void resetRange();
  virtual void reset() override;
  virtual bool isClearable() {return false;}

private:
  void occupancyGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  /// Resolve the 2D transform between the incoming OccupancyGrid frame and
  /// the costmap global frame. Returns false when the transform is not
  /// available (wrong frame, TF not yet up) - the layer then leaves the
  /// costmap untouched instead of writing misaligned data.
  bool resolveAlignment();

  bool enabled_;
  bool need_recalculation_;
  bool rolling_window_;
  double min_x_, min_y_, max_x_, max_y_;
  int threshold_;
  std::string traversability_topic_;

  /// Costmap global frame (e.g. "map").
  std::string global_frame_;
  /// Frame the incoming OccupancyGrid is expressed in. Empty -> use the
  /// frame_id carried by the message.
  std::string grid_frame_;
  /// What to write for cells without terrain information.
  UnknownPolicy unknown_policy_{UnknownPolicy::kNoInformation};
  /// Grid <-> global frame alignment, valid only when alignment_valid_.
  GridAlignment alignment_;
  bool alignment_valid_{false};

  nav_msgs::msg::OccupancyGrid::SharedPtr latest_occupancy_grid_;
  std::mutex data_mutex_;
  nav2_costmap_2d::Costmap2D * costmap_;
  
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_sub_;

  std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;

};
}  // namespace nav2_traversability_plugin

#endif  // NAV2_TRAVERSABILITY_PLUGIN__TRAVERSABILITY_LAYER_HPP_