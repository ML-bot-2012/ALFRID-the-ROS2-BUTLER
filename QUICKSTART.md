# ALFRID Quick Start Guide

## Prerequisites
- Pi5 with Ubuntu 22.04 + ROS2 Jazzy
- Pi3B+ with Ubuntu 22.04 + ROS2 Humble
- Both on same WiFi network
- RPLidar A1 connected to Pi5
- Motors and encoders connected to Pi3B+
- 12V battery charged

## Network Setup

### On Both Pi5 and Pi3B+
```bash
# Add to ~/.bashrc
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
echo "export ROS_LOCALHOST_ONLY=0" >> ~/.bashrc
source ~/.bashrc
```

### Verify Network
```bash
# From Pi5, ping Pi3B+
ping ALFRIDCL

# From Pi3B+, ping Pi5
ping alfridros
```

## 5-Terminal Quick Start

### Terminal 1: Pi3B+ - Motors & Encoders
```bash
ssh malharlabade@ALFRIDCL
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
ros2 launch butler_gpio pi3b_launch.py
# Expected output:
# [motor_control_node-1] Motor Control Node started
# [encoder_odometry_node-2] Encoder Odometry Node started
```

### Terminal 2: Pi5 - RPLidar
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
ros2 launch rplidar_ros rplidar_a1_launch.py
# Expected output:
# [INFO] Running RplidarNode...
# Should hear RPLidar motor spinning
```

### Terminal 3: Pi5 - Robot State Publisher
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat ~/butler_ros2_ws/src/butler_control/urdf/butler.urdf)"
# Expected output:
# [INFO] robot_state_publisher: Publishing transforms
```

### Terminal 4: Pi5 - SLAM Mapping
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
ros2 launch slam_toolbox online_async_launch.py odom_topic:=encoder_odom
# Expected output:
# [INFO] Initialize completed
# Map should start building as you move robot
```

### Terminal 5: Pi5 - RViz2 Visualization
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
ros2 run rviz2 rviz2
# RViz2 window opens
```

### Optional Terminal 6: Pi5 - Teleop Control
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
python3 ~/butler_ros2_ws/src/butler_control/butler_control/teleop_node.py
# Control with:
# W = Forward
# S = Backward  
# A = Left Turn
# D = Right Turn
# SPACE = Stop
# Q = Quit
```

## RViz2 Setup (if not already configured)

### Add Displays
1. **Robot Model**
   - Click "Add" → Search "RobotModel" → OK
   - Description Topic: `/robot_description`

2. **LiDAR Scan**
   - Click "Add" → Search "LaserScan" → OK
   - Topic: `/scan`

3. **SLAM Map**
   - Click "Add" → Search "Map" → OK
   - Topic: `/map`

4. **Odometry**
   - Click "Add" → Search "Odometry" → OK
   - Topic: `/encoder_odom`

5. **Transforms (Optional)**
   - Click "Add" → Search "TF" → OK

### Save Configuration
- File → Save Config As
- Name: `alfrid_config.rviz`
- Location: `~/alfrid_config.rviz`

## Verify Everything Works

### Check Topics
```bash
ros2 topic list
# Should see:
/cmd_vel
/encoder_odom
/scan
/map
/odom
/parameter_events
/robot_description
/rosout
/tf
/tf_static
```

### Check Nodes
```bash
ros2 node list
# Should see at least:
/motor_control
/encoder_odometry
/rplidar
/slam_toolbox
/rviz2
/robot_state_publisher
```

### Test Movement
```bash
# Publish test command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# Robot should move forward slowly
# RViz2 should show movement and map updates
```

## Mapping Workflow

### Step 1: Move Robot Around
- Use Terminal 6 (teleop) to move robot
- Move slowly (~0.1 m/s)
- Cover entire area you want to map
- Make loops to close the map

### Step 2: Monitor in RViz2
- Watch blue robot model move
- Watch red/green laser points
- Watch gray/black map build
- Check for drift

### Step 3: Save Map
```bash
ros2 run nav2_map_server map_saver_cli -f ~/alfrid_maps/my_map

# Creates:
# ~/alfrid_maps/my_map.pgm (map image)
# ~/alfrid_maps/my_map.yaml (metadata)
```

### Step 4: View Map
```bash
# Convert to PNG
convert ~/alfrid_maps/my_map.pgm ~/alfrid_maps/my_map.png

# View image
eog ~/alfrid_maps/my_map.png
```

## Troubleshooting

### No Movement
```bash
# Check Pi3B+ motors are running
ssh malharlabade@ALFRIDCL
ros2 topic echo /cmd_vel
# Should show movement commands

# Test motors directly
sudo python3 ~/test_motor.py
```

### No LiDAR Data
```bash
# Check RPLidar is connected
ls /dev/ttyUSB*
# Should show /dev/ttyUSB0

# Check RPLidar motor is spinning (listen for noise)
# Restart RPLidar
pkill -f rplidar
sleep 2
ros2 launch rplidar_ros rplidar_a1_launch.py
```

### Map Not Updating
```bash
# Move robot more (SLAM needs motion)
# Check encoder_odom is publishing
ros2 topic echo /encoder_odom

# Restart SLAM
pkill -f slam_toolbox
sleep 2
ros2 launch slam_toolbox online_async_launch.py odom_topic:=encoder_odom
```

### Network Issues
```bash
# Check ROS_DOMAIN_ID is same on both
echo $ROS_DOMAIN_ID  # Should be 0

# Restart ROS2 daemon
ros2 daemon stop
sleep 2
ros2 daemon start

# Verify network
ping ALFRIDCL
ping alfridros
```

### RViz2 Displays Not Showing
```bash
# Make sure topics exist
ros2 topic list | grep -E "scan|map|robot_description|encoder_odom"

# Restart RViz2
pkill -f rviz2
sleep 2
ros2 run rviz2 rviz2 -d ~/alfrid_config.rviz
```

## Shutdown Procedure

### Proper Shutdown (in order)
1. Terminal 6: Press Q (quit teleop)
2. Terminal 5: Press Ctrl+C (stop RViz2)
3. Terminal 4: Press Ctrl+C (stop SLAM)
4. Terminal 3: Press Ctrl+C (stop Robot State Publisher)
5. Terminal 2: Press Ctrl+C (stop RPLidar)
6. Terminal 1: Press Ctrl+C (stop Motors/Encoders)

## Useful Quick Commands
```bash
# View current position
ros2 topic echo /encoder_odom --once

# View map
ros2 topic echo /map --once

# List all running nodes
ros2 node list

# List all topics
ros2 topic list

# Test encoder counts
sudo python3 ~/test_encoders.py

# Test motor control
sudo python3 ~/test_motor.py

# Save and load config in RViz2
# File → Save Config As / Load Config
```

## Common Issues & Quick Fixes

| Issue | Fix |
|-------|-----|
| No /dev/ttyUSB0 | Reconnect RPLidar USB cable |
| GPIO Permission Denied | `sudo pkill -9 python3 && sudo reboot` |
| Map Not Updating | Move robot faster/farther |
| Network Not Working | `export ROS_LOCALHOST_ONLY=0` |
| RViz2 Displays Missing | Create displays in RViz2 window |
| Motors Not Spinning | Check battery voltage (12V) |
| Encoder Counts Wrong | Check GPIO connections |

## Performance Expectations

- **Map Update Rate**: 2-10 Hz
- **Robot Speed**: 0.05-0.15 m/s (slow for SLAM)
- **Map Accuracy**: ±50-100mm
- **CPU Usage**: 40-60% on Pi5
- **Memory**: 200-300 MB

## Next Steps

- [ ] Save your first map
- [ ] Add loop closure detection (advanced SLAM)
- [ ] Implement Nav2 for autonomous navigation
- [ ] Add IMU for better heading estimation
- [ ] Implement battery monitoring
- [ ] Create custom launch files

## Support & Debugging

### Enable Debug Output
```bash
export RCL_LOGGING_LEVEL_rcl=DEBUG
ros2 launch slam_toolbox online_async_launch.py odom_topic:=encoder_odom
```

### Check System Health
```bash
# CPU/Memory usage
top

# Disk space
df -h

# Network latency
ping -c 5 alfridros
```

### Save Debug Logs
```bash
ros2 run rclpy rclpy --log-level DEBUG > alfrid_debug.log 2>&1 &
```

## Video Recording

### Record RViz2
```bash
# Install ffmpeg
sudo apt install -y ffmpeg

# Record screen
ffmpeg -f x11grab -i :0 -r 30 ~/mapping_video.mp4
# Press 'q' to stop
```

Property of 5KROBOTICS & MALHAR LABADE © 2025
Copy entire text above and:

Click "Add file" → "Create new file"
Name it: QUICKSTART.md
Paste everything
Click "Commit changes"

Done! All hostnames used instead of IPs! 🎉🚀no, when you say pi_ip and stuff say pi_hostname and tell them to create hostname as ALFRIDROS and ALFRIDCL for pi5 and pi3B5:25 PM# ALFRID Quick Start Guide

## Prerequisites
- Pi5 with Ubuntu 22.04 + ROS2 Jazzy
- Pi3B+ with Ubuntu 22.04 + ROS2 Humble
- Both on same WiFi network
- RPLidar A1 connected to Pi5
- Motors and encoders connected to Pi3B+
- 12V battery charged

## Hostname Setup

### Set Hostname on Pi5
```bash
sudo hostnamectl set-hostname ALFRIDROS
sudo reboot
```

### Set Hostname on Pi3B+
```bash
sudo hostnamectl set-hostname ALFRIDCL
sudo reboot
```

### Verify Hostnames
```bash
# On each Pi, check hostname
hostname
# Pi5 should show: ALFRIDROS
# Pi3B+ should show: ALFRIDCL
```

## Network Setup

### On Both ALFRIDROS (Pi5) and ALFRIDCL (Pi3B+)
```bash
# Add to ~/.bashrc
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
echo "export ROS_LOCALHOST_ONLY=0" >> ~/.bashrc
source ~/.bashrc
```

### Verify Network
```bash
# From ALFRIDROS (Pi5), ping ALFRIDCL
ping ALFRIDCL

# From ALFRIDCL (Pi3B+), ping ALFRIDROS
ping ALFRIDROS
```

## 5-Terminal Quick Start

### Terminal 1: ALFRIDCL (Pi3B+) - Motors & Encoders
```bash
ssh malharlabade@ALFRIDCL
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
ros2 launch butler_gpio pi3b_launch.py
# Expected output:
# [motor_control_node-1] Motor Control Node started
# [encoder_odometry_node-2] Encoder Odometry Node started
```

### Terminal 2: ALFRIDROS (Pi5) - RPLidar
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
ros2 launch rplidar_ros rplidar_a1_launch.py
# Expected output:
# [INFO] Running RplidarNode...
# Should hear RPLidar motor spinning
```

### Terminal 3: ALFRIDROS (Pi5) - Robot State Publisher
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat ~/butler_ros2_ws/src/butler_control/urdf/butler.urdf)"
# Expected output:
# [INFO] robot_state_publisher: Publishing transforms
```

### Terminal 4: ALFRIDROS (Pi5) - SLAM Mapping
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
ros2 launch slam_toolbox online_async_launch.py odom_topic:=encoder_odom
# Expected output:
# [INFO] Initialize completed
# Map should start building as you move robot
```

### Terminal 5: ALFRIDROS (Pi5) - RViz2 Visualization
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
ros2 run rviz2 rviz2
# RViz2 window opens
```

### Optional Terminal 6: ALFRIDROS (Pi5) - Teleop Control
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
python3 ~/butler_ros2_ws/src/butler_control/butler_control/teleop_node.py
# Control with:
# W = Forward
# S = Backward  
# A = Left Turn
# D = Right Turn
# SPACE = Stop
# Q = Quit
```

## RViz2 Setup (if not already configured)

### Add Displays
1. **Robot Model**
   - Click "Add" → Search "RobotModel" → OK
   - Description Topic: `/robot_description`

2. **LiDAR Scan**
   - Click "Add" → Search "LaserScan" → OK
   - Topic: `/scan`

3. **SLAM Map**
   - Click "Add" → Search "Map" → OK
   - Topic: `/map`

4. **Odometry**
   - Click "Add" → Search "Odometry" → OK
   - Topic: `/encoder_odom`

5. **Transforms (Optional)**
   - Click "Add" → Search "TF" → OK

### Save Configuration
- File → Save Config As
- Name: `alfrid_config.rviz`
- Location: `~/alfrid_config.rviz`

## Verify Everything Works

### Check Topics
```bash
ros2 topic list
# Should see:
/cmd_vel
/encoder_odom
/scan
/map
/odom
/parameter_events
/robot_description
/rosout
/tf
/tf_static
```

### Check Nodes
```bash
ros2 node list
# Should see at least:
/motor_control
/encoder_odometry
/rplidar
/slam_toolbox
/rviz2
/robot_state_publisher
```

### Test Movement
```bash
# Publish test command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# Robot should move forward slowly
# RViz2 should show movement and map updates
```

## Mapping Workflow

### Step 1: Move Robot Around
- Use Terminal 6 (teleop) to move robot
- Move slowly (~0.1 m/s)
- Cover entire area you want to map
- Make loops to close the map

### Step 2: Monitor in RViz2
- Watch blue robot model move
- Watch red/green laser points
- Watch gray/black map build
- Check for drift

### Step 3: Save Map
```bash
ros2 run nav2_map_server map_saver_cli -f ~/alfrid_maps/my_map

# Creates:
# ~/alfrid_maps/my_map.pgm (map image)
# ~/alfrid_maps/my_map.yaml (metadata)
```

### Step 4: View Map
```bash
# Convert to PNG
convert ~/alfrid_maps/my_map.pgm ~/alfrid_maps/my_map.png

# View image
eog ~/alfrid_maps/my_map.png
```

## Troubleshooting

### No Movement
```bash
# Check ALFRIDCL (Pi3B+) motors are running
ssh malharlabade@ALFRIDCL
ros2 topic echo /cmd_vel
# Should show movement commands

# Test motors directly
sudo python3 ~/test_motor.py
```

### No LiDAR Data
```bash
# Check RPLidar is connected
ls /dev/ttyUSB*
# Should show /dev/ttyUSB0

# Check RPLidar motor is spinning (listen for noise)
# Restart RPLidar
pkill -f rplidar
sleep 2
ros2 launch rplidar_ros rplidar_a1_launch.py
```

### Map Not Updating
```bash
# Move robot more (SLAM needs motion)
# Check encoder_odom is publishing
ros2 topic echo /encoder_odom

# Restart SLAM
pkill -f slam_toolbox
sleep 2
ros2 launch slam_toolbox online_async_launch.py odom_topic:=encoder_odom
```

### Network Issues
```bash
# Check ROS_DOMAIN_ID is same on both
echo $ROS_DOMAIN_ID  # Should be 0

# Restart ROS2 daemon
ros2 daemon stop
sleep 2
ros2 daemon start

# Verify network
ping ALFRIDCL
ping ALFRIDROS
```

### RViz2 Displays Not Showing
```bash
# Make sure topics exist
ros2 topic list | grep -E "scan|map|robot_description|encoder_odom"

# Restart RViz2
pkill -f rviz2
sleep 2
ros2 run rviz2 rviz2 -d ~/alfrid_config.rviz
```

## Shutdown Procedure

### Proper Shutdown (in order)
1. Terminal 6: Press Q (quit teleop)
2. Terminal 5: Press Ctrl+C (stop RViz2)
3. Terminal 4: Press Ctrl+C (stop SLAM)
4. Terminal 3: Press Ctrl+C (stop Robot State Publisher)
5. Terminal 2: Press Ctrl+C (stop RPLidar)
6. Terminal 1: Press Ctrl+C (stop Motors/Encoders)

## Useful Quick Commands
```bash
# View current position
ros2 topic echo /encoder_odom --once

# View map
ros2 topic echo /map --once

# List all running nodes
ros2 node list

# List all topics
ros2 topic list

# Test encoder counts
sudo python3 ~/test_encoders.py

# Test motor control
sudo python3 ~/test_motor.py

# Save and load config in RViz2
# File → Save Config As / Load Config
```

## Common Issues & Quick Fixes

| Issue | Fix |
|-------|-----|
| No /dev/ttyUSB0 | Reconnect RPLidar USB cable |
| GPIO Permission Denied | `sudo pkill -9 python3 && sudo reboot` |
| Map Not Updating | Move robot faster/farther |
| Network Not Working | `export ROS_LOCALHOST_ONLY=0` |
| RViz2 Displays Missing | Create displays in RViz2 window |
| Motors Not Spinning | Check battery voltage (12V) |
| Encoder Counts Wrong | Check GPIO connections |

## Performance Expectations

- **Map Update Rate**: 2-10 Hz
- **Robot Speed**: 0.05-0.15 m/s (slow for SLAM)
- **Map Accuracy**: ±50-100mm
- **CPU Usage**: 40-60% on ALFRIDROS (Pi5)
- **Memory**: 200-300 MB

## Next Steps

- [ ] Save your first map
- [ ] Add loop closure detection (advanced SLAM)
- [ ] Implement Nav2 for autonomous navigation
- [ ] Add IMU for better heading estimation
- [ ] Implement battery monitoring
- [ ] Create custom launch files

## Support & Debugging

### Enable Debug Output
```bash
export RCL_LOGGING_LEVEL_rcl=DEBUG
ros2 launch slam_toolbox online_async_launch.py odom_topic:=encoder_odom
```

### Check System Health
```bash
# CPU/Memory usage
top

# Disk space
df -h

# Network latency
ping -c 5 ALFRIDROS
```

### Save Debug Logs
```bash
ros2 run rclpy rclpy --log-level DEBUG > alfrid_debug.log 2>&1 &
```

## Video Recording

### Record RViz2
```bash
# Install ffmpeg
sudo apt install -y ffmpeg

# Record screen
ffmpeg -f x11grab -i :0 -r 30 ~/mapping_video.mp4
# Press 'q' to stop
```

Property of 5KROBOTICS & MALHAR LABADE © 2025
