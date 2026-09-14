# ROS 2 Autonomous Robot Navigation 🤖

A ROS 2-based autonomous robot navigation and RViz2 visualization prototype designed as the software foundation for a Raspberry Pi 4 and LiDAR-based mobile robot.

## 📌 Project Overview

This project demonstrates the basic software structure of an autonomous mobile robot using ROS 2 and Python. It includes simulated mapping, robot movement, path planning, goal visualization, and LiDAR-style scan data.

The system is currently developed as a simulation prototype and will later be integrated with real hardware such as:

- Raspberry Pi 4 Model B
- RPLIDAR sensor
- ESP32
- L298N motor driver
- DC geared motors
- IMU and ultrasonic sensors

## ✨ Features

- ROS 2 Python node
- Simulated 2D occupancy grid map
- Robot position visualization
- Planned path visualization
- Goal position marker
- Simulated LiDAR scan data
- RViz2 visualization
- Foundation for SLAM and autonomous navigation
- Future Raspberry Pi and hardware integration

## 🛠️ Technologies Used

- ROS 2 Lyrical
- Python 3
- RViz2
- Ubuntu 26.04
- WSL 2
- ROS 2 topics
- OccupancyGrid
- Path
- LaserScan
- MarkerArray

## 📂 Project Structure

```text
robot_demo/
│
├── demo_map.py
├── robot_slam.rviz
└── README.md
