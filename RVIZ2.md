# RViz2 Setup & Configuration for ALFRID

## What is RViz2?
RViz2 is the ROS2 3D visualization tool. It displays robot model, sensor data (LiDAR), maps, and transforms in real-time.

## Installation on Pi5

### Install RViz2
```bash
sudo apt install -y ros-jazzy-rviz2
```

## Quick Start

### Launch RViz2
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
ros2 run rviz2 rviz2
```

## Display Configuration

### Add Robot Model
1. Click "Add" button (bottom left)
2. Search for "RobotModel"
3. Click OK
4. Set Description Topic to `/robot_description`

### Add LiDAR Data
1. Click "Add"
2. Search for "LaserScan"
3. Click OK
4. Set Topic to `/scan`
5. You should see red/green laser points

### Add SLAM Map
1. Click "Add"
2. Search for "Map"
3. Click OK
4. Set Topic to `/map`
5. You should see the occupancy grid

### Add Odometry
1. Click "Add"
2. Search for "Odometry"
3. Click OK
4. Set Topic to `/encoder_odom`

### Add TF (Transform Frames)
1. Click "Add"
2. Search for "TF"
3. Click OK
4. You should see frame axes (red=X, green=Y, blue=Z)

## Display Settings

### Change Background Color
- Top menu → View → Background Color
- Set to black or white for better visibility

### Change Robot Model Color
- Left panel → RobotModel → Alpha (0.0-1.0)
- Adjust transparency

### Change Point Cloud Size
- Left panel → LaserScan → Size (pixels)
- Increase for larger points

### Change Map Display
- Left panel → Map → Color Scheme (costmap/map)
- Adjust Alpha for transparency

## Save Configuration

### Export Config
1. File → Save Config As
2. Name: `alfrid_config.rviz`
3. Save location: `~/alfrid_config.rviz`

### Load Saved Config
```bash
ros2 run rviz2 rviz2 -d ~/alfrid_config.rviz
```

## ROS2 Topics Used in RViz2
/robot_description  - URDF model
/scan              - LiDAR laser scans
/map               - SLAM occupancy map
/encoder_odom      - Encoder odometry
/tf                - Transform frames
/tf_static         - Static transforms

## Troubleshooting

### Robot Model Not Appearing
```bash
# Check if robot_description is being published
ros2 topic list | grep robot_description

# Verify robot_state_publisher is running
ros2 node list | grep robot_state
```

### LiDAR Points Not Showing
```bash
# Check if /scan topic exists
ros2 topic list | grep scan

# View scan data
ros2 topic echo /scan --once
```

### Map Not Displaying
```bash
# Check if SLAM is running
ros2 node list | grep slam

# Check /map topic
ros2 topic list | grep map
```

### Transforms Not Showing
```bash
# Check TF tree
ros2 run tf2_tools view_frames

# View specific transform
ros2 run tf2_ros tf2_echo odom base_link
```

## Tips & Tricks

### Pan Camera
- Hold Middle Mouse Button + Drag

### Rotate Camera
- Hold Right Mouse Button + Drag

### Zoom
- Scroll Wheel
- Or Middle Mouse Button Scroll

### Select Objects
- Click on objects in 3D view
- See properties in left panel

### Reset View
- View → Default Camera → Reset

### Record Video
```bash
# Install video tools
sudo apt install -y ffmpeg

# Record RViz window
ffmpeg -f x11grab -i :0 -r 30 ~/rviz_recording.mp4
```

## Full Setup Example

### Terminal 1 - Launch RPLidar
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch rplidar_ros rplidar_a1_launch.py
```

### Terminal 2 - Launch SLAM
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch slam_toolbox online_async_launch.py odom_topic:=encoder_odom
```

### Terminal 3 - Launch Robot State Publisher
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat ~/butler_ros2_ws/src/butler_control/urdf/butler.urdf)"
```

### Terminal 4 - Launch RViz2
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
ros2 run rviz2 rviz2 -d ~/alfrid_config.rviz
```

## Performance Notes
- RViz2 uses CPU for rendering
- Reduce point cloud size if slow
- Hide unnecessary displays
- Update rate: 10-30 Hz recommended

## Default RViz2 Layout
Left Panel: Display properties
3D View: Main visualization
Top Menu: File, Edit, View, Tools, Help

Property of 5KROBOTICS & MALHAR LABADE © 2025
