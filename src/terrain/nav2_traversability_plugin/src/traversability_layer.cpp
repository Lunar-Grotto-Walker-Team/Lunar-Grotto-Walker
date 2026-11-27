#include "nav2_traversability_plugin/traversability_layer.hpp"

#include "nav2_traversability_plugin/terrain_cost.hpp"

#include <cmath>
#include <string>

namespace nav2_traversability_plugin
{

namespace
{
/// Yaw (rotation around z) of a quaternion.
double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}
}  // namespace
TraversabilityLayer::TraversabilityLayer()
{
}

void TraversabilityLayer::resetRange()
{
  min_x_ = min_y_ =  std::numeric_limits<double>::max();
  max_x_ = max_y_ = -std::numeric_limits<double>::max();
}

void TraversabilityLayer::onInitialize()
{
  auto node = node_.lock();
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("threshold", rclcpp::ParameterValue(0)); // Define the threshold parameter
  declareParameter("traversability_topic", rclcpp::ParameterValue("/traversability/traversability_map"));
  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "threshold", threshold_);
  node->get_parameter(name_ + "." + "traversability_topic", traversability_topic_);

  std::string unknown_policy_str;
  declareParameter("unknown_policy", rclcpp::ParameterValue(std::string("no_information")));
  declareParameter("grid_frame", rclcpp::ParameterValue(std::string("")));
  node->get_parameter(name_ + "." + "unknown_policy", unknown_policy_str);
  node->get_parameter(name_ + "." + "grid_frame", grid_frame_);
  unknown_policy_ = unknownPolicyFromString(unknown_policy_str);

  costmap_ = layered_costmap_->getCostmap();
  global_frame_ = layered_costmap_->getGlobalFrameID();

  // The TF buffer has to exist before the subscription is created: the
  // callback resolves the grid -> global frame transform right away.
  tfBuffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

  occupancy_grid_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    traversability_topic_, 10, std::bind(&TraversabilityLayer::occupancyGridCallback, this, std::placeholders::_1));

  need_recalculation_ = true;
  current_ = true;

  rolling_window_ = layered_costmap_->isRolling();
}

void TraversabilityLayer::occupancyGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> guard(data_mutex_);
  latest_occupancy_grid_ = msg;
  resolveAlignment();
}

bool TraversabilityLayer::resolveAlignment()
{
  alignment_valid_ = false;
  if (!latest_occupancy_grid_) {
    return false;
  }
  const auto & grid = *latest_occupancy_grid_;
  const std::string source_frame = grid_frame_.empty() ? grid.header.frame_id : grid_frame_;
  if (source_frame.empty()) {
    return false;
  }

  // Transform of the grid frame into the costmap global frame.
  double tx = 0.0, ty = 0.0, yaw_tf = 0.0;
  if (source_frame != global_frame_) {
    try {
      const auto tf = tfBuffer_->lookupTransform(
        global_frame_, source_frame, rclcpp::Time(0), rclcpp::Duration::from_seconds(0.2));
      tx = tf.transform.translation.x;
      ty = tf.transform.translation.y;
      yaw_tf = yawFromQuaternion(tf.transform.rotation);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("nav2_traversability_plugin"),
        *node_.lock()->get_clock(), 5000,
        "Cannot transform terrain grid frame '%s' into costmap frame '%s': %s",
        source_frame.c_str(), global_frame_.c_str(), ex.what());
      return false;
    }
  }

  // The OccupancyGrid origin is expressed in its own frame -> global frame.
  const double gx = grid.info.origin.position.x;
  const double gy = grid.info.origin.position.y;
  const double yaw_grid = yawFromQuaternion(grid.info.origin.orientation);
  const double c = std::cos(yaw_tf), s = std::sin(yaw_tf);
  const double origin_x = tx + c * gx - s * gy;
  const double origin_y = ty + s * gx + c * gy;

  alignment_valid_ = alignment_.setFromTransform(
    origin_x, origin_y, yaw_tf + yaw_grid, grid.info.resolution);
  return alignment_valid_;
}

void TraversabilityLayer::updateBounds(double robot_x, double robot_y, double /*robot_yaw*/, double* min_x, double* min_y, double* max_x, double* max_y)
{
    if (!enabled_ || !latest_occupancy_grid_ || !alignment_valid_) {
        return;
    }

    if (layered_costmap_->isRolling()) {
      updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
    }

    // Axis aligned bounding box (in the costmap global frame) of the whole
    // occupancy grid.  The previous implementation used the raw origin and a
    // hard-coded factor 4 on the upper bound, which inflated the costmap
    // update window by 4x and ignored the grid's actual extent.
    double grid_min_x = 0.0, grid_min_y = 0.0, grid_max_x = 0.0, grid_max_y = 0.0;
    alignment_.gridBounds(
      static_cast<int>(latest_occupancy_grid_->info.width),
      static_cast<int>(latest_occupancy_grid_->info.height),
      grid_min_x, grid_min_y, grid_max_x, grid_max_y);

    *min_x = std::min(*min_x, grid_min_x);
    *min_y = std::min(*min_y, grid_min_y);
    *max_x = std::max(*max_x, grid_max_x);
    *max_y = std::max(*max_y, grid_max_y);
}

void TraversabilityLayer::updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  std::lock_guard<std::mutex> guard(data_mutex_);
  if (!enabled_ || !latest_occupancy_grid_ || !alignment_valid_) {
    return;
  }

  // Get dimensions of the occupancy grid
  const auto& info = latest_occupancy_grid_->info;
  int width = info.width;
  int height = info.height;

  // Get the pointer to the master costmap array
  unsigned char* costmap_array = master_grid.getCharMap();

  // Iterate through the specified window
  for (int i = min_i; i < max_i; ++i) {
    for (int j = min_j; j < max_j; ++j) {
      // Convert costmap coordinates to occupancy grid coordinates
      double wx, wy;
      master_grid.mapToWorld(i, j, wx, wy);

      // world -> grid cell index, through the resolved grid alignment
      // (handles a grid frame that differs from the costmap global frame).
      int mx = 0, my = 0;
      if (!alignment_.worldToGrid(wx, wy, mx, my)) {
        continue;
      }

      // Check bounds
      if (mx >= 0 && mx < width && my >= 0 && my < height) {
        int index = my * width + mx;

        // The bridge emits Nav2 Costmap2D-internal cost bytes
        // (0 / 85 UNKNOWN / 128 SOFT / 254 LETHAL).  nav_msgs/OccupancyGrid
        // .data is declared int8[], so the bytes MUST be read as UNSIGNED:
        // as int8, 254 becomes -2 and 128 becomes -128, which silently
        // degrades every lethal / soft cell to FREE, while the UNKNOWN
        // marker 85 becomes ">= 65" and turns into a lethal obstacle -
        // exactly inverted.
        const uint8_t raw_value = static_cast<uint8_t>(latest_occupancy_grid_->data[index]);
        const TerrainCost terrain = terrainByteToCost(raw_value, unknown_policy_);
        if (!terrain.has_cost) {
          // Policy "skip": no terrain information for this cell - leave it to
          // the other costmap layers.
          continue;
        }

        // Calculate the index in the master costmap array
        int costmap_index = master_grid.getIndex(i, j);
        // Set the cost in the master costmap array
        costmap_array[costmap_index] = terrain.cost;
      }
    }
  }

  updateWithMax(master_grid, min_i, min_j, max_i, max_j);
}
  

void TraversabilityLayer::onFootprintChanged()
{
  need_recalculation_ = true;

  RCLCPP_DEBUG(rclcpp::get_logger(
      "nav2_costmap_2d"), "GradientLayer::onFootprintChanged(): num footprint points: %lu",
    layered_costmap_->getFootprint().size());
}

void TraversabilityLayer::reset()
{
  std::lock_guard<std::mutex> guard(data_mutex_);
  latest_occupancy_grid_.reset();
}
}  // namespace nav2_traversability_plugin

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_traversability_plugin::TraversabilityLayer, nav2_costmap_2d::Layer)

