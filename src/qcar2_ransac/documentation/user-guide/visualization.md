# Visualization

## Traffic Sign Colors

Each traffic sign class has a distinct color in RViz2:

| Sign Class | Color |
|------------|-------|
| stop | Red |
| yield | Yellow |
| round | Teal |
| speed | Blue |
| traffic | Green |
| cross | Orange |
| no_entry | Purple |
| parking | Cyan |
| one_way | Magenta |
| pedestrian | Olive |
| default | Gray |

## Display Format

Confirmed objects are displayed as spheres with text labels showing the class name and sequential ID (e.g., "Stop_1", "Stop_2").

## RViz2 Setup

Add the following MarkerArray displays:

| Topic | Description | Frame |
|-------|-------------|-------|
| `/confirmed_markers` | Confirmed object positions and labels | map |
| `/object_markers` | Current detection visualization | base_link |
| `/test/markers` | Python node confirmed visualization | map |
| `/test/object_markers` | Python node current detections | base_link |
