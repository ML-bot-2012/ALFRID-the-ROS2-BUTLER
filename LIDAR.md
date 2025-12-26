# RPLidar A1 Setup & Configuration

## Hardware Specs
- Model: RPLidar A1
- Range: 12 meters max
- FOV: 270 degrees
- Resolution: 1-3 degrees
- Rotation Speed: 5.5 Hz (330 RPM)
- Serial Number: 9AC4FA86C2E392D0A5E59FF70D105C60

## Connection

### USB Connection (Recommended for Pi5)
USB-C Cable → Pi5 USB Port
Device Path: /dev/ttyUSB0
Baud Rate: 115200

### Serial Connection (Alternative)
TX Pin → GPIO 15 (Pi5 UART TX)
RX Pin → GPIO 14 (Pi5 UART RX)
GND → Pi5 GND

## Installation on Pi5

### 1. Install RPLidar ROS2 Driver
```bash
sudo apt install -y ros-jazzy-rplidar-ros
```

### 2. Check Connection
```bash
ls /dev/ttyUSB*
# Should show: /dev/ttyUSB0
```

### 3. Grant Permissions
```bash
sudo usermod -a -G dialout $USER
sudo chmod 666 /dev/ttyUSB0
```

### 4. Launch RPLidar
```bash
ros2 launch rplidar_ros rplidar_a1_launch.py
```

## Verify LiDAR Data

### Check Topic
```bash
ros2 topic list | grep scan
# Should show: /scan
```

### View Scan Data
```bash
ros2 topic echo /scan --once
# Should show laser scan points
```

### Visualize in RViz2
1. Open RViz2: `ros2 run rviz2 rviz2`
2. Add Display → LaserScan
3. Set Topic to `/scan`
4. You should see red/green laser points

## Troubleshooting

### No /dev/ttyUSB0
```bash
# Check if device is recognized
lsusb
# Should show RPLidar device

# Try reconnecting USB cable
# Or check: dmesg | tail -20
```

### Permission Denied
```bash
# Give permission
sudo chmod 666 /dev/ttyUSB0

# Or add to dialout group
sudo usermod -a -G dialout $USER
newgrp dialout
```

### Scan Data Not Updating
```bash
# Check if motor is spinning (should hear it)
# Kill and restart rplidar launch
pkill -f rplidar
sleep 2
ros2 launch rplidar_ros rplidar_a1_launch.py
```

### RPLidar Spinning But No Data
```bash
# Check baud rate
ros2 launch rplidar_ros rplidar_a1_launch.py serial_baudrate:=115200

# Restart ROS2 daemon
ros2 daemon stop
sleep 2
ros2 daemon start
```

## ROS2 Topic Details

### /scan Topic (sensor_msgs/msg/LaserScan)
frame_id: laser_frame
angle_min: -3.14159 (rad)
angle_max: 3.14159 (rad)
angle_increment: 0.017453 (1 degree)
ranges: [distance_1, distance_2, ..., distance_360]

## Performance Notes
- Update Rate: ~10 Hz
- Range Accuracy: ±30mm at 1-6m
- Max Rotation Speed: 10 Hz (optional)
- Power: 5V 1A (from Pi5 USB)

## SLAM Integration
RPLidar data feeds into SLAM Toolbox for real-time mapping:
```bash
ros2 launch slam_toolbox online_async_launch.py
```

Property of 5KROBOTICS & MALHAR LABADE © 2025

Copy and paste the text above:

Click "Add file" → "Create new file"
Name it: LIDAR.md
Paste the content
Click "Commit changes"

Ready for File 3 (RViz)? 🚀for everything abt lidar give me 1 cand p file5:14 PM# RPLidar A1 Complete Setup & Configuration

## Hardware Specs
- Model: RPLidar A1
- Range: 12 meters max
- FOV: 270 degrees
- Resolution: 1-3 degrees
- Rotation Speed: 5.5 Hz (330 RPM)
- Serial Number: 9AC4FA86C2E392D0A5E59FF70D105C60

## Connection

### USB Connection (Recommended for Pi5)
USB-C Cable → Pi5 USB Port
Device Path: /dev/ttyUSB0
Baud Rate: 115200

### Serial Connection (Alternative)
TX Pin → GPIO 15 (Pi5 UART TX)
RX Pin → GPIO 14 (Pi5 UART RX)
GND → Pi5 GND

## Installation on Pi5

### 1. Install RPLidar ROS2 Driver
```bash
sudo apt install -y ros-jazzy-rplidar-ros
```

### 2. Check Connection
```bash
ls /dev/ttyUSB*
# Should show: /dev/ttyUSB0
```

### 3. Grant Permissions
```bash
sudo usermod -a -G dialout $USER
sudo chmod 666 /dev/ttyUSB0
```

### 4. Launch RPLidar
```bash
ros2 launch rplidar_ros rplidar_a1_launch.py
```

## Verify LiDAR Data

### Check Topic
```bash
ros2 topic list | grep scan
# Should show: /scan
```

### View Scan Data
```bash
ros2 topic echo /scan --once
# Should show laser scan points
```

### Visualize in RViz2
1. Open RViz2: `ros2 run rviz2 rviz2`
2. Add Display → LaserScan
3. Set Topic to `/scan`
4. You should see red/green laser points

## Troubleshooting

### No /dev/ttyUSB0
```bash
# Check if device is recognized
lsusb
# Should show RPLidar device

# Try reconnecting USB cable
# Or check: dmesg | tail -20
```

### Permission Denied
```bash
# Give permission
sudo chmod 666 /dev/ttyUSB0

# Or add to dialout group
sudo usermod -a -G dialout $USER
newgrp dialout
```

### Scan Data Not Updating
```bash
# Check if motor is spinning (should hear it)
# Kill and restart rplidar launch
pkill -f rplidar
sleep 2
ros2 launch rplidar_ros rplidar_a1_launch.py
```

### RPLidar Spinning But No Data
```bash
# Check baud rate
ros2 launch rplidar_ros rplidar_a1_launch.py serial_baudrate:=115200

# Restart ROS2 daemon
ros2 daemon stop
sleep 2
ros2 daemon start
```

## ROS2 Topic Details

### /scan Topic (sensor_msgs/msg/LaserScan)
frame_id: laser_frame
angle_min: -3.14159 (rad)
angle_max: 3.14159 (rad)
angle_increment: 0.017453 (1 degree)
ranges: [distance_1, distance_2, ..., distance_360]

## Performance Notes
- Update Rate: ~10 Hz
- Range Accuracy: ±30mm at 1-6m
- Max Rotation Speed: 10 Hz (optional)
- Power: 5V 1A (from Pi5 USB)

## SLAM Integration
RPLidar data feeds into SLAM Toolbox for real-time mapping:
```bash
ros2 launch slam_toolbox online_async_launch.py
```

Property of 5KROBOTICS & MALHAR LABADE © 2025
