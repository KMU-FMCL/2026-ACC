# 2026-ACC — QCar2 Autonomous Driving Stack

ROS 2 autonomy stack for the Quanser QCar2 miniature autonomous vehicle platform. Developed as a research prototype and entry for the 2026 ACC (Autonomous Car Competition) at Kookmin University FMCL.

Targets both QLabs simulation and real QCar2 hardware from a single codebase — map files and launch arguments differ, but no code changes are required when switching targets.

## Requirements

| Dependency | Version |
|---|---|
| Ubuntu | 22.04 LTS |
| ROS 2 | Humble Hawksbill |
| C++ | 17 |
| CMake | 3.25+ |
| Python | 3.10+ |

ROS 2 package dependencies: `robot_localization`, `cartographer_ros`, `nav2`, `tf2_ros`, `cv_bridge`, `image_transport`

Python dependencies: `numpy`, `scipy`, `opencv-python`, `ultralytics` (YOLOv11)

## Repository Structure

```
ros2_ws/
├── src/
│   ├── qcar2_nodes/               # Hardware drivers: CSI cam, RGBD, LiDAR, motor
│   ├── qcar2_msgs/                # Custom message types (Object3D, LateralGuidance, PhQuintic, ...)
│   ├── qcar2_interfaces/          # Service and action interfaces
│   ├── qcar2_transforms/          # Coordinate transformation utilities (Python)
│   ├── qcar2_ground_truth_tf/     # QLabs ground truth TF publisher (map->odom->base_link)
│   ├── qcar2_ransac/              # RANSAC-based 3D object localizer (C++)
│   ├── acc_object_localizer/      # Traffic sign localization via RealSense D435 depth
│   ├── qcar2_planner/             # PH quintic curve path planning library (C++)
│   ├── qcar2_control/             # VFG guidance + PID controller (C++)
│   ├── qcar2_object_detection/    # YOLOv11s 2D detection [submodule]
│   ├── qcar2_lane_detection/      # Lane boundary detection [submodule]
│   ├── qcar2_perception_bringup/  # Perception launch orchestration [submodule]
│   ├── qcar2_localization/        # EKF + Cartographer SLAM pipeline [submodule]
│   └── Lanelet2/                  # HD map library [submodule]
├── map/                           # Occupancy grid map (map.pgm + map.yaml)
├── qcar_map.pbstream              # Cartographer localization map — real QCar2
├── qlabs_map.pbstream             # Cartographer localization map — QLabs sim
└── yolov11s.pt                    # YOLOv11s detection model weights
```

Submodules (`qcar2_object_detection`, `qcar2_lane_detection`, `qcar2_perception_bringup`, `qcar2_localization`) are hosted at `github.com/KMU-FMCL`. Do not modify them in-tree — submit changes upstream.

## Setup

Clone with submodules:

```bash
git clone --recurse-submodules <repo-url> ros2_ws
cd ros2_ws
```

If already cloned without submodules:

```bash
git submodule update --init --recursive
```

The setup script installs all required tools:

```bash
# Run the setup script
sh ./src/qcar2_perception_bringup/setting/setup.sh

# Verify installation
sh ./src/qcar2_perception_bringup/setting/verify.sh

```

## Build

Build all packages from the workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

Build a single package:

```bash
colcon build --packages-select <package_name>
```

## Test

Run all tests:

```bash
source /opt/ros/humble/setup.bash
colcon test --ctest-args '--output-on-failure'
colcon test-result --verbose
```

Run tests for a single package:

```bash
colcon test --packages-select <package_name>
```

Run with C++ linting enforced (example: `qcar2_planner`):

```bash
colcon build --packages-select qcar2_planner --cmake-args -DENFORCE_FORMAT=ON -DENFORCE_LINT=ON
```

## Stack Overview

The autonomy stack is divided into four layers: perception, localization, planning, and control.

### Perception

**`qcar2_nodes`** — Hardware abstraction layer. Publishes sensor data from the LiDAR, RealSense RGBD camera, CSI cameras (front/rear/left/right), and the QCar2 motor interface.

**`qcar2_object_detection`** — YOLOv11s 2D bounding box detection on CSI camera frames.

**`qcar2_lane_detection`** — Lane boundary detection from CSI camera images.

**`qcar2_ransac`** — Fuses YOLO detections with RealSense depth to localize detected objects in 3D using RANSAC. Outputs `qcar2_msgs/Object3DArray` in the map frame.

**`acc_object_localizer`** — Localizes traffic signs detected by YOLO to global coordinates using RealSense D435 depth and TF lookups.

### Localization

**`qcar2_localization`** — EKF (via `robot_localization`) fused with wheel odometry and IMU, combined with Google Cartographer in localization-only mode against a pre-built `.pbstream` map.

**`qcar2_ground_truth_tf`** — Publishes ground truth `map->odom->base_link` TF from QLabs simulation state (used in sim in place of Cartographer).

**`qcar2_transforms`** — Python library for coordinate frame math utilities (quaternion, frame transforms, waypoint utilities).

### Planning

**`qcar2_planner`** — Pythagorean Hodograph (PH) quintic curve library. Computes analytically exact arc lengths, unit normals, and constant or variable-distance offset curves via RBF interpolation. The PH curves are published as `qcar2_msgs/PhQuinticPath` and consumed by the control layer.

### Control

**`qcar2_control`** — Vector Field Guidance (VFG) for lateral steering + PID for longitudinal speed control. Two configurations are available:

- **Bezier mode** (`bezier_vfg_control.launch.py`): path defined by Bezier control points
- **PH mode** (`ph_vfg_control.launch.py`): path defined by PH quintic segments from `qcar2_planner`

The controller pipeline:

```
path topic (BezierCurve or PhQuinticPath)
        |
        v
  vfg_guidance_node  <--  TF (map -> base_link)
        |
        v
  /vfg/lateral_guidance (LateralGuidance)
        |
        v
  pid_controller_node
        |
        v
  /qcar2_motor_speed_cmd (MotorCommands)
```

## Launch Reference

### QLabs Simulation

Start QCar2 virtual sensor nodes:

```bash
ros2 launch qcar2_nodes qcar2_virtual_launch.py
```

Start localization (EKF + Cartographer, localization mode):

```bash
ros2 launch qcar2_localization localization_ekf_launch.py \
  use_sim_time:=true \
  load_state_filename:=/path/to/ros2_ws/qlabs_map.pbstream
```

Or use ground truth TF (sim only, no SLAM):

```bash
ros2 launch qcar2_ground_truth_tf ground_truth_tf_publisher_launch.py
```

### Real QCar2

Start hardware sensor nodes:

```bash
ros2 launch qcar2_nodes qcar2_launch.py
```

Start localization against the real-world map:

```bash
ros2 launch qcar2_localization localization_ekf_launch.py \
  load_state_filename:=/path/to/ros2_ws/qcar_map.pbstream
```

### Perception

```bash
ros2 launch qcar2_perception_bringup bringup.launch.py
ros2 launch qcar2_ransac ransac_localizer_launch.py
ros2 launch acc_object_localizer object_localizer.launch.py
```

### Planning and Control

PH curve mode (recommended):

```bash
ros2 launch qcar2_control ph_vfg_control.launch.py
```

Bezier mode:

```bash
ros2 launch qcar2_control bezier_vfg_control.launch.py
```

### SLAM (map building)

Build a new map on real hardware:

```bash
ros2 launch qcar2_nodes qcar2_cartographer_launch.py
```

Build a new map in QLabs:

```bash
ros2 launch qcar2_nodes qcar2_cartographer_virtual_launch.py
```

## Key Message Types

Defined in `qcar2_msgs`:

| Message | Fields |
|---|---|
| `Object3D` | `class_name`, `class_id`, `confidence`, `position` (map frame), `distance` |
| `Object3DArray` | array of `Object3D` |
| `LateralGuidance` | heading error, cross-track error, curvature, VFG vectors, lookahead curvature, status |
| `PhQuintic` | G1 Hermite data (start/end point + tangent), branch, arc length |
| `PhQuinticPath` | array of `PhQuintic` segments |
| `BezierCurve` | Bezier control points |

## Configuration

RANSAC localizer parameters are in `src/qcar2_ransac/config/params.yaml`. Key parameters:

| Parameter | Default | Description |
|---|---|---|
| `min_depth` / `max_depth` | 0.2 / 1.0 m | Valid depth range for detections |
| `signal_color_max_depth` | 2.0 m | Extended depth range for signal color classes (red/yellow/green) |
| `tracking_distance` | 1.0 m | Max distance to associate detection with existing tracker |
| `min_measurements` | 5 | Minimum detections before a tracker is confirmed |
| `std_threshold` | 0.3 m | Max XY standard deviation for confirmation |
| `dynamic_update_alpha` | 0.4 | EMA ratio for dynamic class position updates |
| `dynamic_timeout_sec` | 1.0 s | Remove dynamic tracker if unseen for this long |
| `hold_position_sec` | 10.0 s | Sample-and-hold interval for non-excluded classes |
| `process_rate` | 10.0 Hz | Node update rate |

EKF and Cartographer parameters are in `src/qcar2_localization/config/`.

VFG and PID tuning parameters are described in `src/qcar2_control/README.md`.

## Development

Code style: `clang-format` and `clang-tidy` for C++, `flake8` and `pep257` for Python. All linter rules are enforced in CI.

Commit format: `<type>(<scope>): <description>` (semantic commits, no AI attribution).

Branch from `develop` for features: `git checkout -b feat/<name>`. Open a PR to `develop`; squash merge after approval.

Test coverage for `qcar2_planner` must remain at or above 80 %.

## License

Mixed licenses per package:

- `qcar2_planner`, `qcar2_ransac`: GPL-3.0-only
- `qcar2_nodes`, `qcar2_transforms`, `qcar2_ground_truth_tf`: Apache-2.0
- `acc_object_localizer`: MIT
- `qcar2_control`, `qcar2_msgs`, `qcar2_interfaces`: not yet declared

See individual `package.xml` and `LICENSE` files for details.
