#include <chrono>
#include <memory>
#include <cmath>
 

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  // Initialize publisher
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  // Initialize timer
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&MapMemoryNode::updateMap, this));

  global_map_.header.frame_id = "sim_world";
  global_map_.info.resolution = 0.1;
  global_map_.info.width = 300;
  global_map_.info.height = 300;
  global_map_.info.origin.position.x = -15.0;
  global_map_.info.origin.position.y = -15.0;
  global_map_.info.origin.position.z = 0.0;
  global_map_.info.origin.orientation.w = 1.0;
  global_map_.data.resize(300 * 300, -1);
  global_map_.header.stamp = this->now();
  last_x = 0.0;
  last_y = 0.0;
  for(int i = 0; i < 300*300; ++i){
    global_map_.data[i] = -1;
  }
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  // Instead of publishing on timer callback if costmap is updated, we will publish based on costmapcallback if
  // timer threshold is passed. This gets rid of rotational errors due to delay
  if(!distance_threshold_passed){
    return;
  }
  if(!timer_threshold_passed){
    return;
  }
  latest_costmap_ = *msg;
  integrateCostmap();
  map_pub_->publish(global_map_);
  distance_threshold_passed = false;
  timer_threshold_passed = false;
}
// Callback for odometry updates
void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;

  const geometry_msgs::msg::Quaternion& orientation = msg->pose.pose.orientation;
  double roll, pitch, yaw;
        tf2::Quaternion quat;
        tf2::fromMsg(orientation, quat);
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  // Compute distance traveled
  double distance = std::sqrt(std::pow(x - last_x, 2) + std::pow(y - last_y, 2));
  if (distance >= 0.5) { // distance threshold for updating
    last_x = x;
    last_y = y;
    last_yaw = yaw;
    distance_threshold_passed = true;
  }
}

// Timer-based map update
void MapMemoryNode::updateMap() {
  timer_threshold_passed = true;
}

// Integrate the latest costmap into the global map
void MapMemoryNode::integrateCostmap() {
  // Get the robot's pose in the global frame from the latest odometry data
  double robot_x = last_x;
  double robot_y = last_y;
  double robot_yaw = last_yaw;

  // Iterate through the costmap and transform each point to the global frame
  for (size_t y = 0; y < latest_costmap_.info.height; ++y) {
    for (size_t x = 0; x < latest_costmap_.info.width; ++x) {
      int8_t cost = latest_costmap_.data[y * latest_costmap_.info.width + x];
      if (cost >= 0) { // Only consider known cells
        // Convert costmap cell coordinates to robot frame coordinates
        double cell_x = x * latest_costmap_.info.resolution + latest_costmap_.info.origin.position.x;
        double cell_y = y * latest_costmap_.info.resolution + latest_costmap_.info.origin.position.y;

        // Transform robot frame coordinates to global frame coordinates
        double global_x = robot_x + cell_x * cos(robot_yaw) - cell_y * sin(robot_yaw);
        double global_y = robot_y + cell_x * sin(robot_yaw) + cell_y * cos(robot_yaw);

        // Convert global frame coordinates to global map cell coordinates
        int global_cell_x = (global_x - global_map_.info.origin.position.x) / global_map_.info.resolution;
        int global_cell_y = (global_y - global_map_.info.origin.position.y) / global_map_.info.resolution;

        // Update the global map with the transformed cost, because we did some transformations inflate with 1 radius.
        if (global_cell_x >= 0 && global_cell_x < global_map_.info.width &&
            global_cell_y >= 0 && global_cell_y < global_map_.info.height) {

          for(int i = -1; i < 2; ++i){
            for(int j = -1; j < 2; ++j){
              if(global_cell_x + i >= 0 && global_cell_x + i < global_map_.info.width &&
                 global_cell_y + j >= 0 && global_cell_y + j < global_map_.info.height){
                   global_map_.data[(global_cell_y+j) * global_map_.info.width + global_cell_x+i] = cost;

                 }
            }
          }
        }
      }
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
