# Configuration

All parameters are configured in `/config/params.yaml`.
Changes take effect immediately on relaunch without rebuild.

## Camera Intrinsics (640x480)

### RGB Camera

| Parameter | Value | Description |
|-----------|-------|-------------|
| `rgb_fx` | 455.2 | Focal length x |
| `rgb_fy` | 459.43 | Focal length y |
| `rgb_cx` | 308.53 | Principal point x |
| `rgb_cy` | 213.56 | Principal point y |

### Depth Camera

| Parameter | Value | Description |
|-----------|-------|-------------|
| `depth_fx` | 385.6 | Focal length x |
| `depth_fy` | 385.6 | Focal length y |
| `depth_cx` | 321.9 | Principal point x |
| `depth_cy` | 237.3 | Principal point y |

## RANSAC Algorithm

| Parameter | Default | Description |
|-----------|---------|-------------|
| `ransac_iterations` | 100 | Number of iterations |
| `ransac_threshold` | 0.08 | Inlier threshold in meters |
| `min_inliers_ratio` | 0.3 | Minimum ratio of inliers (0-1) |
| `min_valid_points` | 10 | Minimum valid depth points in ROI |

## Depth Processing

| Parameter | Default | Description |
|-----------|---------|-------------|
| `min_depth` | 0.2 | Minimum valid depth (meters) |
| `max_depth` | 5.0 | Maximum valid depth (meters) |
| `depth_scale` | 0.001 | Conversion factor from raw depth to meters (mono16 = mm) |

## Bounding Box ROI

| Parameter | Default | Description |
|-----------|---------|-------------|
| `bbox_scale_min` | 0.3 | Minimum scale factor (30% of bbox) |
| `bbox_scale_max` | 0.5 | Maximum scale factor (50% of bbox) |

The actual ROI uses the average: `(bbox_scale_min + bbox_scale_max) / 2 = 40%` of the bounding box centered on the detection.

## Object Tracking

| Parameter | Default | Description |
|-----------|---------|-------------|
| `tracking_distance` | 1.0 | Max distance to associate with existing tracker (meters) |
| `min_measurements` | 5 | Minimum measurements before confirmation |
| `std_threshold` | 0.3 | Max standard deviation for confirmation (meters) |
| `moving_avg_window` | 10 | Window size for position averaging |
| `duplicate_distance` | 0.5 | Distance threshold for duplicate removal (meters) |

## Processing

| Parameter | Default | Description |
|-----------|---------|-------------|
| `process_rate` | 10.0 | Processing rate (Hz) |
