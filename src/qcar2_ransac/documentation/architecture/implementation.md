# Implementation Details

## Depth Estimation Strategy

The RANSAC algorithm treats depth estimation as a model-fitting problem:

1. Randomly sample depth values from ROI
2. Count inliers within threshold distance
3. Select model with most inliers
4. Return median of inliers for robustness

This approach handles noisy QLabs depth camera data effectively.

## Object Tracking Strategy

The tracker maintains separate lists for each object class:

- New measurements are associated with nearest existing tracker within `tracking_distance`
- Trackers accumulate measurements in a moving window
- Confirmation requires `min_measurements` with standard deviation below `std_threshold`
- Confirmed objects are persistent and receive sequential IDs
- Duplicate detection prevents multiple confirmations of same physical object

## Python vs C++ Implementation

Both implementations follow the same algorithm:

| Implementation | File | Use Case |
|---------------|------|----------|
| C++ node | `ransac_localizer_node` | Production use, optimized performance |
| Python node | `test_localizer.py` | Testing and validation, easier debugging |

Choose based on your development needs.
