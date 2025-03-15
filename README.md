# Project Overview

This project is a comprehensive implementation of a robotic system using ROS2 (Robot Operating System 2). The system includes various nodes that handle different aspects of robot navigation, such as path planning, costmap generation, control, and map memory. The project showcases advanced algorithms and techniques used in robotics, including A* pathfinding, costmaps, map fusion, ray tracing, pure pursuit control, and differential drive.

## Project Structure

The project is organized into several directories, each containing specific components of the robotic system:

- **planner**: Contains the path planning node.
- **costmap**: Contains the costmap generation node.
- **control**: Contains the control node.
- **map_memory**: Contains the map memory node.
- **docker**: Contains Dockerfiles for containerizing the nodes.
- **watod_scripts**: Contains scripts for setting up the development and Docker environments.

## Key Algorithms and Techniques

### A* Pathfinding Algorithm

The A* algorithm is used in the path planning node to find the optimal path from the robot's current position to the goal. The algorithm is modified to include costs for proximity to costly areas, ensuring that the robot avoids high-risk regions. The heuristic function used is the Euclidean distance between the current node and the goal.

### Bresenham's Line Algorithm

Bresenham's line algorithm is implemented in the costmap node for ray tracing. This algorithm efficiently determines which cells in a grid should be traversed to form a close approximation to a straight line between two points. It is used to mark free space in the costmap. Ray tracing is implemented here to separate known space from unknown space: all grids are marked unknown, and we ray-trace from the robot until we reach the costmap edge or a cost point to form known space.

### Pure Pursuit Control

The pure pursuit algorithm is implemented in the control node to follow the planned path. This algorithm computes the necessary linear and angular velocities to steer the robot towards a lookahead point on the path. The lookahead point is dynamically chosen based on the robot's current position and a predefined lookahead distance.

### Differential Drive

The differential drive model is used to control the robot's movement. This model calculates the velocities of the left and right wheels based on the desired linear and angular velocities. The control node publishes velocity commands to the robot's drive system to achieve the desired motion.

### Costmaps

Costmaps are used to represent the environment around the robot. The costmap node generates a local costmap based on LIDAR data, marking obstacles and free space. The map memory node integrates these local costmaps into a global map, allowing the robot to navigate in a larger environment. The costmap includes inflation of obstacles to account for the robot's size and safety margin.

## Skills and Techniques Learned

### ROS2 Development

- Creating and managing ROS2 nodes.
- Implementing publishers, subscribers, and timers.
- Using ROS2 message types for communication between nodes.

### Advanced Algorithms

- Implementing and modifying the A* pathfinding algorithm.
- Using Bresenham's line algorithm for efficient grid traversal.
- Applying ray tracing techniques for obstacle detection.
- Implementing pure pursuit control for path following.

### Robotics Concepts

- Understanding and applying the differential drive model for robot control.
- Generating and managing costmaps for obstacle avoidance and navigation.
- Integrating local costmaps into a global map for comprehensive environment representation.

### Software Engineering Practices

- Organizing code into modular and reusable components.
- Using Docker for containerizing and deploying ROS2 nodes.
- Writing clean and maintainable code with proper documentation.

## Conclusion

This project demonstrates a comprehensive understanding of advanced robotics algorithms and techniques, as well as proficiency in ROS2 development. The implementation of path planning, costmap generation, control, and map memory nodes showcases the ability to create a robust and efficient robotic system. The skills and techniques learned from this project are valuable for developing complex robotic applications and solving real-world navigation challenges.