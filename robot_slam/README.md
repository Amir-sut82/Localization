# Robot SLAM Package

A custom 2D SLAM (Simultaneous Localization and Mapping) implementation for ROS2, designed for differential-drive robots with 2D LiDAR sensors.

## Overview

This package implements a complete SLAM system **from scratch** without using existing SLAM packages like `slam_toolbox`, `gmapping`, or `Cartographer`. It is designed as an educational implementation demonstrating core SLAM concepts.

### Key Features

- **Odometry-based Motion Model**: Predicts robot motion from wheel odometry
- **Scan-to-Map Matching**: Corrects pose drift using gradient descent optimization
- **Log-odds Occupancy Grid Mapping**: Builds probabilistic 2D maps using Bresenham ray tracing
- **Map Saving**: Exports maps to standard `.pgm` and `.yaml` formats

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                        SLAM Node                            │
│  ┌─────────────┐  ┌──────────────┐  ┌───────────────────┐  │
│  │   Motion    │  │    Scan      │  │   Occupancy Grid  │  │
│  │   Model     │──│   Matcher    │──│       Map         │  │
│  └─────────────┘  └──────────────┘  └───────────────────┘  │
│         ▲               ▲                    │              │
│         │               │                    ▼              │
│    /ekf/odom        /scan              /map topic           │
└─────────────────────────────────────────────────────────────┘
                                               │
                                               ▼
┌─────────────────────────────────────────────────────────────┐
│                    Map Saver Node                           │
│           Saves to .pgm and .yaml files                     │
└─────────────────────────────────────────────────────────────┘
```

## Algorithm Details

### 1. Motion Model (Odometry-based)

Uses the standard odometry motion model for differential drive robots:

```
δrot1 = atan2(y' - y, x' - x) - θ
δtrans = √((x' - x)² + (y' - y)²)
δrot2 = θ' - θ - δrot1
```

Where `(x, y, θ)` and `(x', y', θ')` are consecutive odometry readings.

### 2. Scan Matching

Implements correlation-based scan-to-map matching with gradient descent:

1. Convert laser scan to 2D points in robot frame
2. Transform points to map frame using current pose estimate
3. Compute score (sum of occupancy values at scan endpoints)
4. Use gradient descent to maximize score
5. Apply search window constraints to prevent divergence

### 3. Occupancy Grid Mapping

Uses log-odds representation for numerical stability:

```
l(m|z) = l(m) + log(p(m|z) / (1 - p(m|z)))
```

- **Ray Tracing**: Bresenham's line algorithm marks cells as free
- **Hit Points**: Cells at laser endpoints marked as occupied
- **Log-odds clamping**: Prevents saturation

## Topics

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/ekf/odom` | `nav_msgs/Odometry` | Robot odometry |
| `/scan` | `sensor_msgs/LaserScan` | 2D LiDAR data |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/map` | `nav_msgs/OccupancyGrid` | Generated occupancy grid map |
| `/slam/pose` | `geometry_msgs/PoseStamped` | Estimated robot pose |

### TF Transforms

| Transform | Description |
|-----------|-------------|
| `map` → `odom` | Corrects odometry drift |

## Parameters

### SLAM Node Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `odom_topic` | `/ekf/odom` | Odometry topic name |
| `scan_topic` | `/scan` | Laser scan topic name |
| `map_frame` | `map` | Map frame ID |
| `odom_frame` | `odom` | Odometry frame ID |
| `base_frame` | `base_link` | Robot base frame ID |
| `map_resolution` | `0.05` | Map resolution (m/cell) |
| `map_width` | `50.0` | Map width (meters) |
| `map_height` | `50.0` | Map height (meters) |
| `map_origin_x` | `-25.0` | Map origin X (meters) |
| `map_origin_y` | `-25.0` | Map origin Y (meters) |
| `use_scan_matching` | `true` | Enable scan matching |
| `scan_matching_iterations` | `20` | Max matching iterations |
| `map_update_interval` | `1.0` | Map publish rate (seconds) |

### Map Saver Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `output_dir` | `~/maps` | Directory for saved maps |
| `map_name` | `slam_map` | Base filename for maps |
| `auto_save` | `false` | Enable periodic auto-save |
| `auto_save_interval` | `60.0` | Auto-save interval (seconds) |

## Installation

```bash
# Navigate to your workspace src directory
cd ~/Robot_ws/src

# Copy the robot_slam package
cp -r /path/to/robot_slam .

# Build the package
cd ~/Robot_ws
colcon build --packages-select robot_slam

# Source the workspace
source install/setup.bash
```

## Usage

### Running SLAM

```bash
# Basic launch
ros2 launch robot_slam slam.launch.py

# With custom parameters
ros2 launch robot_slam slam.launch.py \
    map_resolution:=0.05 \
    map_width:=100.0 \
    map_height:=100.0 \
    use_scan_matching:=true
```

### Saving the Map

The map saver node provides an interactive interface:
- Press `s` + Enter to save the map
- Press `q` + Enter to quit

Maps are saved as:
- `slam_map.pgm` - Grayscale image of the map
- `slam_map.yaml` - Metadata file for map_server

### Visualization in RViz

```bash
rviz2 -d $(ros2 pkg prefix robot_slam)/share/robot_slam/rviz/slam.rviz
```

Or manually add:
- Map display on `/map` topic
- LaserScan display on `/scan` topic
- Pose display on `/slam/pose` topic
- TF display

## File Structure

```
robot_slam/
├── CMakeLists.txt
├── package.xml
├── include/robot_slam/
│   ├── slam_node.hpp
│   ├── occupancy_grid_map.hpp
│   ├── scan_matcher.hpp
│   └── motion_model.hpp
├── src/
│   ├── slam_node.cpp
│   ├── occupancy_grid_map.cpp
│   ├── scan_matcher.cpp
│   ├── motion_model.cpp
│   └── map_saver_node.cpp
├── launch/
│   └── slam.launch.py
├── rviz/
│   └── slam.rviz
├── config/
│   └── slam_params.yaml
└── README.md
```

## Limitations

This implementation has several limitations compared to production SLAM systems:

1. **No Loop Closure**: The system does not detect or correct loop closures, leading to accumulated drift in large environments.

2. **Simple Scan Matching**: Uses gradient descent which can get stuck in local minima. Production systems use more sophisticated methods like multi-resolution search or branch-and-bound.

3. **No Pose Graph Optimization**: Does not maintain or optimize a pose graph, which is essential for consistent large-scale mapping.

4. **Fixed Map Size**: The map size is fixed at initialization and cannot grow dynamically.

5. **No Sensor Noise Modeling**: Assumes ideal sensor measurements without explicit noise modeling.

6. **Single-threaded**: All processing runs in callback threads, which may cause latency issues at high update rates.

7. **No Recovery Mechanisms**: If the robot becomes poorly localized, there's no kidnapped robot detection or recovery.

8. **Basic Motion Model**: Uses deterministic odometry integration without probabilistic modeling.

9. **No Landmark-based SLAM**: Only uses raw scan data, not extracted features or landmarks.

10. **Limited to 2D**: Only supports 2D horizontal laser scans, not 3D LiDAR.

## Dependencies

- ROS2 (tested on Humble)
- Eigen3
- nav_msgs
- sensor_msgs
- geometry_msgs
- tf2, tf2_ros, tf2_geometry_msgs

## License

MIT License

## Author

Student - Robotics Course Assignment
