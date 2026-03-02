# Coordinate Transforms

## RGB → Depth Extrinsic

Hardcoded transformation matrix from RGB camera frame to depth camera frame:

```
[[   1        0.004008     0.0001655   -0.01474   ]
 [ -0.004007  1           -0.003435   -0.0004152 ]
 [ -0.0001792 0.003434     1          -0.0002451 ]
 [   0        0            0           1         ]]
```

## Depth Camera → base_link Extrinsic

Hardcoded transformation matrix from depth camera frame to vehicle base_link frame:

```
[[  0   0   1   0.095 ]
 [ -1   0   0   0.032 ]
 [  0  -1   0   0.172 ]
 [  0   0   0   1     ]]
```

Translation components (in meters):
- X: 0.095 (forward)
- Y: 0.032 (left)
- Z: 0.172 (up)

## TF Tree

The system relies on the following TF transforms:

```
map → base_link (provided by Cartographer)
      ↓
base_link → camera_depth_frame (static, hardcoded)
      ↓
camera_depth_frame → camera_color_frame (static, hardcoded)
```
