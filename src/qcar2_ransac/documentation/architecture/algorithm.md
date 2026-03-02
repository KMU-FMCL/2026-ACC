# Algorithm Pipeline

## Overview

The RANSAC localizer processes YOLO detections through the following pipeline:

```
YOLO Detection → RGB→Depth Transform → ROI Extraction → RANSAC Depth →
Camera→Base Transform → Base→Map Transform → Object Tracking → Confirmation
```

## Pipeline Steps

### 1. Coordinate Transformation

Transform YOLO bounding box from RGB frame to depth frame using camera extrinsics.

### 2. ROI Extraction

Extract depth values from center region of bounding box (30-50% of bbox size).

### 3. Depth Filtering

Filter depth values within valid range (0.2m - 5.0m) and remove invalid readings.

### 4. RANSAC Estimation

Apply RANSAC algorithm to estimate robust depth value from noisy measurements.

### 5. Camera → base_link Transform

Convert depth camera coordinates to vehicle base_link frame using calibrated extrinsic matrix.

### 6. base_link → map Transform

Transform to global map coordinates using TF2 (Cartographer publishes map→base_link).

### 7. Object Tracking

Track objects over time, associate new measurements with existing trackers based on distance.

### 8. Confirmation

Confirm object positions when sufficient measurements (≥5) are collected with low standard deviation (<0.3m).

### 9. Duplicate Removal

Prevent multiple confirmed objects of same class within duplicate_distance threshold.

### 10. Visualization

Publish color-coded markers with labels (e.g., "Stop_1", "Yield_2") for RViz2.
