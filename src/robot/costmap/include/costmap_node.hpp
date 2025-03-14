#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

 
#include "costmap_core.hpp"
 
class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();
    
    // Place callback function here
    void publishCostmap();
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void inflateObstacles();
    void initializeCostmap();

  private:
    robot::CostmapCore costmap_;
    // Place these constructs here
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    //rclcpp::TimerBase::SharedPtr timer_;
    double array[200][200]; // resolution of 0.1m, 20m x 20m
    int8_t inflated_array[200][200];
};
 
#endif 