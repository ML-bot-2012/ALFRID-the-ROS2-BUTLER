# SLAM Setup & Mapping Code

## What is SLAM?
SLAM = Simultaneous Localization and Mapping
Real-time map building while tracking robot position using LiDAR data.

## Hardware Requirements
- RPLidar A1 (connected to Pi5)
- Encoder odometry from Pi3B+
- Pi5 (ROS2 Jazzy)

## SLAM Toolbox Installation

### On Pi5
```bash
source /opt/ros/jazzy/setup.bash
sudo apt install -y ros-jazzy-slam-toolbox
```

## SLAM Configuration

### Create slam_config.yaml
```yaml
slam_toolbox:
  plugins:
    - BoundsProvider: "BoundsProvider"
    - Corrector: "CorrectorPlugin"
    - MotionModel: "OdometryMotionModel"
    - ScanMatcher: "ScanMatcherPlugin"
    - ScanSolver: "CeresScanSolver"
  transforms:
    base_frame: base_link
    map_frame: map
    odom_frame: odom
  scan_topic: /scan
  odom_topic: /encoder_odom
  map_update_interval: 5.0
  resolution: 0.05
  max_laser_range: 12.0
  minimum_travel_heading: 0.5
  minimum_travel_distance: 0.5
  map_start_pose: [0.0, 0.0, 0.0]
```

## Launch SLAM

### Basic SLAM Launch
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

ros2 launch slam_toolbox online_async_launch.py odom_topic:=encoder_odom
```

### With Custom Config
```bash
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=/path/to/slam_config.yaml \
  odom_topic:=encoder_odom
```

## Complete Startup Sequence

### Terminal 1 (Pi3B+ - Motors & Encoders)
```bash
ssh malharlabade@192.168.86.226
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
ros2 launch butler_gpio pi3b_launch.py
```

### Terminal 2 (Pi5 - RPLidar)
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
ros2 launch rplidar_ros rplidar_a1_launch.py
```

### Terminal 3 (Pi5 - Robot State Publisher)
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat ~/butler_ros2_ws/src/butler_control/urdf/butler.urdf)"
```

### Terminal 4 (Pi5 - SLAM)
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
ros2 launch slam_toolbox online_async_launch.py odom_topic:=encoder_odom
```

### Terminal 5 (Pi5 - RViz2 Visualization)
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
ros2 run rviz2 rviz2
```

### Terminal 6 (Pi5 - Teleop Control)
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
python3 ~/butler_ros2_ws/src/butler_control/butler_control/teleop_node.py
# W=Forward, S=Backward, A=Left, D=Right, SPACE=Stop
```

## Mapping Procedure

### Step 1: Verify All Topics
```bash
ros2 topic list
# Should see: /scan, /encoder_odom, /map
```

### Step 2: Start Moving Robot
- Use teleop to move robot around
- Move slowly for better mapping
- Cover the entire area you want to map
- Make loops to close the map

### Step 3: Monitor SLAM
```bash
# Check map updates
ros2 topic echo /map --once

# Check odometry
ros2 topic echo /encoder_odom --once

# View in RViz2 (recommended)
```

### Step 4: Save Map
```bash
ros2 run nav2_map_server map_saver_cli -f ~/alfrid_maps/my_map
# Creates: my_map.pgm (image) and my_map.yaml (metadata)
```

## SLAM Parameters Explained
```yaml
map_update_interval: 5.0          # Seconds between map updates
resolution: 0.05                  # Grid cell size (meters)
max_laser_range: 12.0             # Max LiDAR range to use
minimum_travel_heading: 0.5       # Min rotation (radians) between scans
minimum_travel_distance: 0.5      # Min distance (meters) between scans
map_start_pose: [0.0, 0.0, 0.0]   # Initial position [x, y, theta]
```

## ROS2 Topics Used by SLAM

### Subscribe
- `/scan` (sensor_msgs/msg/LaserScan) - LiDAR data
- `/encoder_odom` (nav_msgs/msg/Odometry) - Encoder odometry

### Publish
- `/map` (nav_msgs/msg/OccupancyGrid) - Occupancy grid map
- `/tf` (tf2_msgs/msg/TFMessage) - Transform map→odom→base_link
- `/odom_updates` (nav_msgs/msg/OccupancyGrid) - Map updates

## Map File Formats

### my_map.yaml (Metadata)
```yaml
image: my_map.pgm
resolution: 0.05
origin: [0.0, 0.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

### my_map.pgm (Image)
- Grayscale image
- White = free space
- Black = occupied space
- Gray = unknown space

## Load Saved Map

### Use in Navigation
```bash
ros2 run nav2_map_server map_server --ros-args -p yaml_filename_:=my_map.yaml
```

### View Map File
```bash
# Convert PGM to PNG for viewing
convert my_map.pgm my_map.png

# View with image viewer
eog my_map.png
```

## Troubleshooting

### SLAM Not Creating Map
```bash
# Check if /scan topic is publishing
ros2 topic echo /scan --once

# Check if /encoder_odom is publishing
ros2 topic echo /encoder_odom --once

# Verify SLAM node is running
ros2 node list | grep slam
```

### Map Is Blank
```bash
# Move robot around more
# SLAM needs motion to create map

# Check robot is actually moving
# Watch /encoder_odom topic for position changes
```

### Poor Map Quality
```bash
# Move robot slower for better scans
# Reduce movement speed in teleop

# Increase LiDAR update rate if possible
# Make sure encoders are accurate

# Clean LiDAR lens
```

### Map Distorted/Twisted
```bash
# Encoder calibration issue
# Verify wheel diameter in encoder node
# Verify wheel base measurement

# Or add IMU for better heading estimation
```

### SLAM Crashes
```bash
# Out of memory - reduce map resolution
# ros2 launch slam_toolbox online_async_launch.py resolution:=0.10

# Or reduce max_laser_range
# ros2 launch slam_toolbox online_async_launch.py max_laser_range:=8.0
```

## Performance Tips

### Faster Mapping
- Increase `map_update_interval` (slower updates)
- Reduce `resolution` (coarser map)
- Decrease `maximum_laser_range`

### Better Accuracy
- Decrease `map_update_interval` (faster updates)
- Increase `resolution` (finer map)
- Slow down robot movement
- Make loops to close map

### Less CPU Usage
- Increase `minimum_travel_distance`
- Increase `minimum_travel_heading`
- Reduce `resolution`

## Map Merging

### Merge Multiple Maps
```bash
python3 ~/butler_ros2_ws/merge_maps.py map1 map2 map3 combined_map
# Creates combined_map.pgm and combined_map.yaml
```

## Advanced SLAM Parameters
```yaml
# Loop closure detection
loop_search_max_linear_distance: 3.0
loop_search_max_angular_distance: 0.785  # ~45 degrees
minimum_score_to_accept_loop_closure: 0.15

# Scan matching
correlation_search_space_dimension: 0.5
correlation_search_space_smearing: 0.03
loop_search_space_dimension: 8.0

# Optimization
solver_iterations: 40
correlation_search_space_resolution: 0.01
```

## Save & Resume Mapping Session

### Save Current Map State
```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: 'checkpoint_1'}}"
```

### Continue Mapping
```bash
# Keep SLAM running, or restart with saved map as reference
ros2 launch slam_toolbox online_async_launch.py \
  map_file_name:=checkpoint_1
```

## ROS2 Services
```bash
# Pause SLAM
ros2 service call /slam_toolbox/pause_new_measurements std_srvs/srv/Empty

# Resume SLAM
ros2 service call /slam_toolbox/unpause_new_measurements std_srvs/srv/Empty

# Clear map
ros2 service call /slam_toolbox/clear_queue std_srvs/srv/Empty
```

## Performance Specifications

- Map Update Rate: 2-10 Hz (configurable)
- CPU Usage: 20-40% on Pi5
- Memory Usage: 100-500 MB (depends on map size)
- Max Map Size: Limited by available RAM
- Accuracy: ±100mm (depends on encoders)

## Future Enhancements

- [ ] Add loop closure detection
- [ ] Save mapping sessions for resumption
- [ ] Implement multi-floor mapping
- [ ] Add graph optimization
- [ ] Web-based map viewer
- [ ] Cloud map storage

## Useful Commands
```bash
# View map in terminal
ros2 topic echo /map

# Save map
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map

# Load map
ros2 run nav2_map_server map_server --ros-args -p yaml_filename_:=my_map.yaml

# View map frames
ros2 run tf2_tools view_frames

# List all nodes
ros2 node list

# List all topics
ros2 topic list

# Echo specific topic
ros2 topic echo /map --once
```

Property of 5KROBOTICS & MALHAR LABADE © 2025
