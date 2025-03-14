#include "planner_node.hpp"

#include <chrono>
#include <cmath>
#include <queue>
#include <unordered_map>

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  // Publisher
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  // Timer
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this)); // changed to 50 sec, see if it's ok or not
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  current_map_ = *msg;
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
      planPath();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  CellIndex goal = worldToGrid(msg->point.x, msg->point.y, current_map_);
  if(current_map_.data[goal.y * current_map_.info.width + goal.x] >0){
    RCLCPP_WARN(this->get_logger(), "Invalid goal, too close to obstacle");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Received goal at %d %d with cost %d", goal.x, goal.y, current_map_.data[goal.y * current_map_.info.width + goal.x]);
  goal_ = *msg;
  goal_received_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_pose_ = msg->pose.pose;
}

void PlannerNode::timerCallback() {
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
      if (goalReached()) {
          RCLCPP_INFO(this->get_logger(), "Goal reached!");
          state_ = State::WAITING_FOR_GOAL;
      }
      //  else {
      //     RCLCPP_INFO(this->get_logger(), "Replanning due to timeout or progress...");
      //     planPath();
      // }
  }
}

bool PlannerNode::goalReached() {
  double dx = goal_.point.x - robot_pose_.position.x;
  double dy = goal_.point.y - robot_pose_.position.y;
  return std::sqrt(dx * dx + dy * dy) < 0.5; // Threshold for reaching the goal
}

void PlannerNode::planPath() {
  if (!goal_received_ || current_map_.data.empty()) {
      RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map or goal!");
      return;
  }

  // A* Implementation
  nav_msgs::msg::Path path;
  path.header.stamp = this->get_clock()->now();
  path.header.frame_id = "sim_world";

  CellIndex start = worldToGrid(robot_pose_.position.x, robot_pose_.position.y, current_map_);
  CellIndex goal = worldToGrid(goal_.point.x, goal_.point.y, current_map_);
  if(current_map_.data[goal.y * current_map_.info.width + goal.x] >0){
    RCLCPP_WARN(this->get_logger(), "Invalid goal, too close to obstacle");
    return;
  }
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  std::unordered_map<CellIndex, double, CellIndexHash> f_score;

  open_set.emplace(start, 0.0);
  g_score[start] = 0.0;
  f_score[start] = heuristic(start, goal);

  while (!open_set.empty()) {
    CellIndex current = open_set.top().index;
    open_set.pop();
    //RCLCPP_INFO(this->get_logger(), "Planning path at %d %d", current.x, current.y);

    if (current == goal) {
        reconstructPath(came_from, current, path);
        path_pub_->publish(path);
        return;
    }

    for (const CellIndex &neighbor : getNeighbors(current, current_map_)) {
        double tentative_g_score = g_score[current] + distance(current, neighbor);

        if(g_score.find(neighbor) == g_score.end()){
          g_score[neighbor] = std::numeric_limits<double>::infinity();
        }
        if (tentative_g_score < g_score[neighbor]) {
            came_from[neighbor] = current;
            g_score[neighbor] = tentative_g_score;
            int c_score = 1000*current_map_.data[neighbor.y * current_map_.info.width + neighbor.x]; // cost of going through high risk areas
            f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal) + c_score;

            open_set.emplace(neighbor, f_score[neighbor]);
        }
    }
  }

  RCLCPP_WARN(this->get_logger(), "Failed to find a path to the goal.");
}

double PlannerNode::heuristic(const CellIndex &a, const CellIndex &b) {
  return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

double PlannerNode::distance(const CellIndex &a, const CellIndex &b) {
  return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

std::vector<CellIndex> PlannerNode::getNeighbors(const CellIndex &current, const nav_msgs::msg::OccupancyGrid &map) {
  std::vector<CellIndex> neighbors;
  for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
          if (dx == 0 && dy == 0) continue;
          CellIndex neighbor(current.x + dx, current.y + dy);
          if (isValid(neighbor, map)) {
              neighbors.push_back(neighbor);
          }
      }
  }
  return neighbors;
}

bool PlannerNode::isValid(const CellIndex &index, const nav_msgs::msg::OccupancyGrid &map) {
  if (index.x < 0 || index.x >= static_cast<int>(map.info.width) ||
      index.y < 0 || index.y >= static_cast<int>(map.info.height)) {
      return false;
  }
  int map_index = index.y * map.info.width + index.x;
  return true;
  //return map.data[map_index] <= 50; // 0 indicates free space. if -1 unknown, still plan through it

  // double radius = 0.7; // radius in meters
  // int cells_radius = static_cast<int>(radius / map.info.resolution);

  // for (int dx = -cells_radius; dx <= cells_radius; ++dx) {
  //     for (int dy = -cells_radius; dy <= cells_radius; ++dy) {
  //         int nx = index.x + dx;
  //         int ny = index.y + dy;

  //         if (nx < 0 || nx >= static_cast<int>(map.info.width) ||
  //             ny < 0 || ny >= static_cast<int>(map.info.height)) {
  //             continue;
  //         }

  //         double distance = std::sqrt(dx * dx + dy * dy) * map.info.resolution;
  //         if (distance <= radius) {
  //             int map_index = ny * map.info.width + nx;
  //             if (map.data[map_index] > 0) { // Occupied cell
  //                 return false;
  //             }
  //         }
  //     }
  // }

  // return true;
}

void PlannerNode::reconstructPath(const std::unordered_map<CellIndex, CellIndex, CellIndexHash> &came_from,
                                  CellIndex current, nav_msgs::msg::Path &path) {
  std::vector<CellIndex> total_path = {current};
  while (came_from.find(current) != came_from.end()) {
      current = came_from.at(current);
      total_path.push_back(current);
  }
  std::reverse(total_path.begin(), total_path.end());

  for (const CellIndex &index : total_path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position = gridToWorld(index, current_map_);
      pose.header.frame_id = "sim_world";
      path.poses.push_back(pose);
  }
}

CellIndex PlannerNode::worldToGrid(double x, double y, const nav_msgs::msg::OccupancyGrid &map) {
  int grid_x = static_cast<int>((x - map.info.origin.position.x) / map.info.resolution);
  int grid_y = static_cast<int>((y - map.info.origin.position.y) / map.info.resolution);
  return CellIndex(grid_x, grid_y);
}

geometry_msgs::msg::Point PlannerNode::gridToWorld(const CellIndex &index, const nav_msgs::msg::OccupancyGrid &map) {
  geometry_msgs::msg::Point point;
  point.x = index.x * map.info.resolution + map.info.origin.position.x;
  point.y = index.y * map.info.resolution + map.info.origin.position.y;
  return point;
}