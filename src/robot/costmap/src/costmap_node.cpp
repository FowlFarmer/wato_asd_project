#include <chrono>
#include <memory>
#include <cmath>
 
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  //timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
}
 
// Define the timer to publish a message every 500ms (from test )
// void CostmapNode::publishMessage() {
//   auto message = std_msgs::msg::String();
//   message.data = "Hello, ROS 2!";
//   RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
//   string_pub_->publish(message);
// }
 

void CostmapNode::initializeCostmap(){
  for (int i = 0; i < 200; ++i) {
    for (int j = 0; j < 200; ++j) {
      array[i][j] = 0;
      inflated_array[i][j] = -1;
    }
  }
  //RCLCPP_INFO(this->get_logger(), "Costmap initialized");
}

void CostmapNode::publishCostmap(){
  auto message = nav_msgs::msg::OccupancyGrid();
  message.header.stamp = this->now();
  message.header.frame_id = "robot/chassis/lidar"; // so the map follows the lidar
  message.info.resolution = 0.1;
  message.info.width = 200;
  message.info.height = 200;

    // Set the origin of the occupancy grid to the center of the costmap
    message.info.origin.position.x = -10.0;  // -10.0 meters (200 cells * 0.1 meters/cell / 2)
    message.info.origin.position.y = -10.0;  // -10.0 meters (200 cells * 0.1 meters/cell / 2)
    message.info.origin.position.z = 0.0;
    message.info.origin.orientation.w = 1.0;
    message.info.origin.orientation.x = 0.0;
    message.info.origin.orientation.y = 0.0;
    message.info.origin.orientation.z = 0.0;

  message.data.resize(200*200);

  for (size_t y = 0; y < 200; ++y) {
      for (size_t x = 0; x < 200; ++x) {
          size_t index = y * 200 + x;
          message.data[index] = inflated_array[x][y];
      }
  }

  costmap_pub_->publish(message);
}

void CostmapNode::rayTrace(int x1, int y1) { // Bresenham's line algorithm (ray trace)
  int x0 = 100;
  int y0 = 100;
  int dx = abs(x1 - x0);
  int dy = abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;

  while (true) {
    
    // if reach known cost, break before editing, if reach edge, edit edge then break
    if (inflated_array[x0][y0] > 0){
      break;
    }
    // Mark the traversed cell as free space (0)
    inflated_array[x0][y0] = 0;

    if(x0 == x1 && y0 == y1){
      break;
    }
    
    int e2 = 2 * err;
    if (e2 > -dy) { 
      err -= dy; 
      x0 += sx; 
    }
    if (e2 < dx) { 
      err += dx; 
      y0 += sy; 
    }
  }
}

void CostmapNode::inflateObstacles(){ // also defines known and unknown space
  for (int i = 0; i < 200; ++i) {
    for (int j = 0; j < 200; ++j) {
      if (array[i][j] == 1) {
        for (int k = i - 10; k < i + 10; ++k) {
          for (int l = j - 10; l < j + 10; ++l) {
            double euclidean_distance = sqrt(pow(((float)i - (float)k)/10, 2) + pow(((float)j - (float)l)/10, 2));
            if (k >= 0 && 
              k < 200 && 
              l >= 0 && 
              l < 200 &&
              euclidean_distance <=1) 
              // first improvement: costmap should be ints between 100-0, not floats from 1-0
              {
              inflated_array[k][l] = (int)100*(1-euclidean_distance) > inflated_array[k][l] ? (int)100*(1-euclidean_distance) : inflated_array[k][l];
            }
          }
        }
      }
    }
  }
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  // Step 1: Initialize costmap
  initializeCostmap();

  // Step 2: Convert LaserScan to grid and mark obstacles (robot is at 100,100)
  // find how many in array

  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double angle = scan->angle_min + i * scan->angle_increment;
      //RCLCPP_INFO(this->get_logger(), "scan->ranges[i]: %f angle %f", scan->ranges[i], angle);

      double x_grid = scan->ranges[i] * cos(angle);
      double y_grid = scan->ranges[i] * sin(angle);
      x_grid *= 10;
      y_grid *= 10;
      x_grid += 100;
      y_grid += 100;
      //RCLCPP_INFO(this->get_logger(), "x_grid: %f, y_grid: %f", x_grid, y_grid);
      if ((int)x_grid >= 0 && (int)x_grid < 200 && (int)y_grid >= 0 && (int)y_grid < 200) {
        array[(int)x_grid][(int)y_grid] = 1;
      }
  }

  // // Step 3: Inflate obstacles
  inflateObstacles();

  for(int i = 0; i < 200; ++i){
    rayTrace(i, 0); // ray trace the edges
    rayTrace(i, 199);
    rayTrace(0, i);
    rayTrace(199, i);
  }

  // // Step 4: Publish costmap
  publishCostmap();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}