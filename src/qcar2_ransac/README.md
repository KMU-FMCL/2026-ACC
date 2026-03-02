# 3D Object Localization - qcar2_ransac

## Changes

- In RANSAC, `red`, `yellow`, and `green` are recognized from 2 m, while other classes are recognized only within 1 m.
- Confirmed object positions now use 10-second sample-and-hold updates for non-excluded classes.

RANSAC-based 3D object localizer for QCar2 autonomous driving in QLabs simulation environment.

## Overview

This ROS2 package localizes traffic signs and objects detected by YOLO in global map coordinates using depth camera information and RANSAC algorithm for robust depth estimation.

**Key Features:**

- RANSAC-based depth estimation for noise-robust localization
- Multi-object tracking with confirmation validation
- Duplicate detection and removal for confirmed objects
- Real-time RViz visualization with color-coded markers
- Configurable parameters via YAML without rebuild
- Both C++ and Python implementations provided

## Quick Start

```bash
# Clone
cd ~/ros2_ws/src
git clone https://git.fmcl.synology.me/2026-ACC/quanser/ros2/platform/3D-Object-localization.git qcar2_ransac

# Build
cd ~/ros2_ws
colcon build --packages-select qcar2_ransac
source install/setup.bash

# Run
ros2 launch qcar2_ransac ransac_localizer_launch.py
```

## Update: YOLO Depth ROI (Central 60%)

`qcar2_od` now supports estimating depth for `yolo_detections` using only the inner bbox ROI.
Default is `0.2 ~ 0.8` on both width/height, which means only the central 60% area is used.

- `use_inner_bbox_depth` (bool): enable inner ROI depth mode
- `bbox_inner_min` (float): start ratio of bbox ROI
- `bbox_inner_max` (float): end ratio of bbox ROI

### CLI (Launch)

```bash
ros2 launch qcar2_od yolo_detector.launch.py \
  use_inner_bbox_depth:=true \
  bbox_inner_min:=0.2 \
  bbox_inner_max:=0.8 \
  depth_scale:=15000.0
```

### CLI (Run)

```bash
ros2 run qcar2_od yolo_detector --ros-args \
  -p use_inner_bbox_depth:=true \
  -p bbox_inner_min:=0.2 \
  -p bbox_inner_max:=0.8 \
  -p depth_scale:=15000.0
```

## Update: 10s Position Hold for Confirmed Objects

For confirmed objects in `qcar2_ransac`, position updates now follow a sample-and-hold policy:

- Classes except `person`, `red`, `green`, `yellow`:
  keep the first confirmed position for 10 seconds, then refresh once, then hold again for 10 seconds.
- Excluded classes (`person`, `red`, `green`, `yellow`):
  continue to update continuously with the existing dynamic behavior.

This policy is applied to confirmed outputs (`/confirmed_objects`, RViz confirmed markers).

### Parameters

- `hold_position_sec` (default: `10.0`):
  hold duration in seconds for non-excluded classes.
- `hold_excluded_classes` (default: `["person", "red", "green", "yellow"]`):
  classes that bypass hold and keep continuous updates.

## Documentation

| Category | Documents |
|----------|-----------|
| **Getting Started** | [Requirements](documentation/getting-started/requirements.md) · [Installation](documentation/getting-started/installation.md) |
| **User Guide** | [Usage](documentation/user-guide/usage.md) · [Configuration](documentation/user-guide/configuration.md) · [Visualization](documentation/user-guide/visualization.md) |
| **Reference** | [Topics](documentation/reference/topics.md) · [Transforms](documentation/reference/transforms.md) |
| **Architecture** | [Algorithm](documentation/architecture/algorithm.md) · [Implementation](documentation/architecture/implementation.md) · [Structure](documentation/architecture/structure.md) |
| **Support** | [Troubleshooting](documentation/support/troubleshooting.md) |

## License

GPL-3.0 (GNU General Public License v3.0)

## Principal Investigator

Suwon Lee (suwon.lee@kookmin.ac.kr) - Director of FMCL

## Maintainer

Taehun Jung (taehun@kookmin.ac.kr)

## Contributors

- Songha Lee (songha327@gmail.com)
- Semin Jeon (eiffeltower1206@kookmin.ac.kr)

Developed at [FMCL (Future Mobility Control Lab)](https://fmcl.kookmin.ac.kr/), Kookmin University

## Version

0.0.1
