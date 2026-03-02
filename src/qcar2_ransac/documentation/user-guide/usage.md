# Usage

## Launch RANSAC Localizer

Launch the C++ node with default parameters:

```bash
ros2 launch qcar2_ransac ransac_localizer_launch.py
```

Launch with simulation time:

```bash
ros2 launch qcar2_ransac ransac_localizer_launch.py use_sim_time:=true
```

Launch with custom parameters:

```bash
ros2 launch qcar2_ransac ransac_localizer_launch.py params_file:=/path/to/custom_params.yaml
```

## Launch Test Localizer (Python)

```bash
ros2 launch qcar2_ransac test_localizer_launch.py
```

## Visualize in RViz2

Add the following displays in RViz2:

- **MarkerArray** on `/confirmed_markers` - confirmed object positions and labels (map frame)
- **MarkerArray** on `/test/markers` - Python node visualization (map frame)
- **MarkerArray** on `/test/object_markers` - current detections (base_link frame)
