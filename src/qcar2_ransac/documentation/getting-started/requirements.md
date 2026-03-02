# System Requirements

## Environment

- ROS2 Humble
- Isaac ROS Docker container (recommended)
- QLabs simulator with QCar2 vehicle
- Ubuntu 24.04

## Hardware

- RealSense depth camera (640x480 resolution)
- QCar2 RC-scale autonomous vehicle (wheelbase: 0.257m)

## Dependencies

- OpenCV
- Eigen3
- TF2
- cv_bridge
- [qcar2_msgs](https://git.fmcl.synology.me/2026-ACC/quanser/ros2/platform/autonomous-driving-msgs.git) (custom message package)
- Cartographer (for map→base_link TF)
