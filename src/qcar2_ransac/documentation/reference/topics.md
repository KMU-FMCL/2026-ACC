# ROS2 Topics

## Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/yolo_detections` | qcar2_msgs/Detection2DArray | YOLO object detection results from RGB camera |
| `/camera/depth_image` | sensor_msgs/Image (mono16) | Depth image from RealSense camera |

## Published Topics

### Main C++ Node

| Topic | Type | Description |
|-------|------|-------------|
| `/object_positions` | qcar2_msgs/Object3DArray | Detected object positions in base_link frame |
| `/confirmed_objects` | qcar2_msgs/Object3DArray | Confirmed object positions in map frame |
| `/object_markers` | visualization_msgs/MarkerArray | Visualization markers for detected objects |
| `/confirmed_markers` | visualization_msgs/MarkerArray | Visualization markers for confirmed objects |

### Python Test Node

| Topic | Type | Description |
|-------|------|-------------|
| `/test/object_positions` | qcar2_msgs/Object3DArray | Detected positions (base_link) |
| `/test/confirmed_objects` | qcar2_msgs/Object3DArray | Confirmed positions (map) |
| `/test/markers` | visualization_msgs/MarkerArray | Confirmed object visualization |
| `/test/object_markers` | visualization_msgs/MarkerArray | Current detection visualization |
