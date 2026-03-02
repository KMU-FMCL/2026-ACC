# Project Structure

```
qcar2_ransac/
├── CMakeLists.txt              # Build configuration
├── package.xml                 # Package manifest
├── README.md                   # Project overview
├── LICENSE                     # GPL-3.0 license
├── CONTRIBUTORS.md             # Contributors list
├── config/
│   └── params.yaml             # Configurable parameters
├── include/
│   └── qcar2_ransac/
│       └── ransac_localizer.hpp # Main header file
├── src/
│   ├── ransac_localizer.cpp    # Core implementation
│   └── ransac_localizer_node.cpp # Node entry point
├── scripts/
│   └── test_localizer.py       # Python test implementation
├── launch/
│   ├── ransac_localizer_launch.py  # Main launch file
│   └── test_localizer_launch.py    # Test node launch file
├── qcar2_ransac/
│   └── __init__.py             # Python package init
└── documentation/
    ├── getting-started/
    │   ├── requirements.md     # System requirements
    │   └── installation.md     # Installation guide
    ├── user-guide/
    │   ├── usage.md            # Usage instructions
    │   ├── configuration.md    # Parameter configuration
    │   └── visualization.md    # RViz visualization
    ├── reference/
    │   ├── topics.md           # ROS2 topics reference
    │   └── transforms.md       # Coordinate transforms
    ├── architecture/
    │   ├── algorithm.md        # Algorithm pipeline
    │   ├── implementation.md   # Implementation details
    │   └── structure.md        # This file
    └── support/
        └── troubleshooting.md  # Troubleshooting guide
```
