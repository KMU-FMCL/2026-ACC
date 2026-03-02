# Troubleshooting

## No Objects Detected

**Symptoms:** No objects appear in `/object_positions` or `/confirmed_objects` topics.

**Solutions:**
- Check that `/yolo_detections` topic is publishing
- Verify `/camera/depth_image` topic has valid data
- Ensure depth values are within min/max range (0.2-5.0m)

```bash
# Check topics
ros2 topic list | grep -E "yolo|depth"
ros2 topic hz /yolo_detections
ros2 topic hz /camera/depth_image
```

## Unstable Positions

**Symptoms:** Object positions fluctuate significantly between frames.

**Solutions:**
- Increase `min_measurements` for more stable confirmation
- Decrease `std_threshold` for stricter confirmation
- Adjust `ransac_threshold` for better outlier rejection

## Duplicate Objects

**Symptoms:** Multiple confirmed objects appear for the same physical object.

**Solutions:**
- Decrease `duplicate_distance` threshold
- Check if multiple objects are actually present in scene

## No TF Transform Available

**Symptoms:** Error messages about missing transforms between frames.

**Solutions:**
- Ensure Cartographer localization is running
- Check that `map` and `base_link` frames exist

```bash
# Verify TF tree
ros2 run tf2_tools view_frames
```

## Common Error Messages

### "No depth data received"

- Verify camera is connected and publishing
- Check depth image topic name matches configuration

### "Transform not available"

- Ensure Cartographer or localization node is running
- Verify TF tree connectivity

### "RANSAC failed to find consensus"

- Increase `ransac_iterations`
- Adjust `ransac_threshold` for current noise level
- Check if object is within valid depth range
