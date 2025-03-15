#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
  lookahead_distance_ = 1.0;  // Lookahead distance
  goal_tolerance_ = 0.1;      // Distance to consider the goal reached
  linear_speed_ = 0.5;        // Constant forward speed

  // Subscribers and Publishers
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = msg;});

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { robot_odom_ = msg; });

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // Timer
  control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), [this]() { controlLoop(); });
}

void ControlNode::controlLoop() {
  // Skip control if no path or odometry data is available
  if (!current_path_ || !robot_odom_) {
      return;
  }

  // Find the lookahead point
  auto lookahead_point = findLookaheadPoint();
  if (!lookahead_point) {
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd_vel);
    return;  // No valid lookahead point found
  }

  // Compute velocity command
  auto cmd_vel = computeVelocity(*lookahead_point);

  // Publish the velocity command
  cmd_vel_pub_->publish(cmd_vel);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {
  geometry_msgs::msg::PoseStamped result;

  if (current_path_->poses.empty()) {
      RCLCPP_WARN(this->get_logger(), "Empty path received.");
      return std::nullopt;
  }

  double total_distance = 0.0;
  const auto& start_pose = current_path_->poses.front().pose.position;

  for (size_t i = 1; i < current_path_->poses.size(); ++i) {
    const auto& prev_pose = current_path_->poses[i - 1].pose.position;
    const auto& curr_pose = current_path_->poses[i].pose.position;

    // Compute Euclidean distance between previous and current pose
    double segment_distance = std::hypot(curr_pose.x - prev_pose.x, curr_pose.y - prev_pose.y);

    if (total_distance + segment_distance >= lookahead_distance_) {
      return current_path_->poses[i]; // return round up pose distance
    }

    total_distance += segment_distance;
  }

  RCLCPP_WARN(this->get_logger(), "Lookahead point not found.");
  return current_path_->poses.back();  // If path is shorter than 1m, return last pose
}

geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped &target) {
  geometry_msgs::msg::Twist cmd_vel;

  // Get the current position and orientation of the robot
  const auto& current_position = robot_odom_->pose.pose.position;
  const auto& current_orientation = robot_odom_->pose.pose.orientation;
  double current_yaw = extractYaw(current_orientation);

  // if distance less than tolerance, return 0 velocity
  if (computeDistance(current_position, target.pose.position) < goal_tolerance_) {
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    RCLCPP_INFO(this->get_logger(), "Control Node: Goal reached!");
    return cmd_vel;
  }

  // Get the target position
  const auto& target_position = target.pose.position;

  // Compute the angle to the target
  double angle_to_target = std::atan2(target_position.y - current_position.y, target_position.x - current_position.x);

  // Compute the heading error
  double heading_error = angle_to_target - current_yaw;

  // Normalize the heading error to the range [-pi, pi]
  while (heading_error > M_PI) heading_error -= 2 * M_PI;
  while (heading_error < -M_PI) heading_error += 2 * M_PI;

  // Compute the linear and angular velocities
  cmd_vel.linear.x = linear_speed_;
  cmd_vel.angular.z = 1.4 * heading_error;  // Gain of 1.4 variable for angular velocity

  return cmd_vel;
}

double ControlNode::computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion &quat) {
  double roll, pitch, yaw;
  tf2::Quaternion quaternion;
  tf2::fromMsg(quat, quaternion);
  tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
  return yaw;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
