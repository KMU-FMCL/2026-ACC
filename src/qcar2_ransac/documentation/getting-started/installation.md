# Installation

## Clone Repository

Clone this repository into your ROS2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://git.fmcl.synology.me/2026-ACC/quanser/ros2/platform/3D-Object-localization.git qcar2_ransac
```

## Install Dependencies

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

## Build Package

```bash
colcon build --packages-select qcar2_ransac
source install/local_setup.bash
```
