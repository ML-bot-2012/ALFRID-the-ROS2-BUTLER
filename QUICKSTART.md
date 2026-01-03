# ALFRID Quick Start Guide

Get ALFRID running in minutes with these commands!

## Prerequisites

- Both Raspberry Pi5 (ALFRIDROS) and Pi3B+ (ALFRIDCL) powered on
- Connected to same network (192.168.86.0/24)
- ROS2 Jazzy on Pi5, ROS2 Humble on Pi3B+
- Cyclone DDS installed on both

---

## 1️⃣ Terminal 1: Motor Control & Encoders (On ALFRIDCL Pi3B+ via SSH)

**Start motor control and encoder odometry:**

```bash
ssh malharlabade@192.168.86.226 "source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=0 && (python3 ~/butler_ros2_ws/src/butler_gpio/butler_gpio/motor_control_node.py & python3 ~/butler_ros2_ws/src/butler_gpio/butler_gpio/encoder_odometry_node.py & wait)"
```

**Or start each separately:**

```bash
# Motor Control
ssh malharlabade@192.168.86.226 "source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=0 && python3 ~/butler_ros2_ws/src/butler_gpio/butler_gpio/motor_control_node.py"

# Encoder Odometry (in separate terminal)
ssh malharlabade@192.168.86.226 "source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=0 && python3 ~/butler_ros2_ws/src/butler_gpio/butler_gpio/encoder_odometry_node.py"
```

---

## 2️⃣ Terminal 2: Navigation Stack (On ALFRIDROS Pi5)

**Start RPLidar, SLAM, and Nav2 with static transforms:**

```bash
source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=0 && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && (
  nohup ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom > /tmp/tf1.log 2>&1 &
  nohup ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link > /tmp/tf2.log 2>&1 &
  nohup ros2 run tf2_ros static_transform_publisher -0.05 0.1 0.0325 0 0 0 base_link left_wheel > /tmp/tf3.log 2>&1 &
  nohup ros2 run tf2_ros static_transform_publisher -0.05 -0.1 0.0325 0 0 0 base_link right_wheel > /tmp/tf4.log 2>&1 &
  nohup ros2 run tf2_ros static_transform_publisher 0.08 0 0.02 0 0 0 base_link caster_wheel > /tmp/tf5.log 2>&1 &
  nohup ros2 run tf2_ros static_transform_publisher 0.08 0 0.2 0 0 0 base_link laser > /tmp/tf6.log 2>&1 &
  sleep 1
  nohup ros2 run rplidar_ros rplidar_composition --ros-args -p serial_port:=/dev/ttyUSB0 -p frame_id:=laser > /tmp/rplidar.log 2>&1 &
  sleep 2
  nohup ros2 run slam_toolbox async_slam_toolbox_node --ros-args -p use_sim_time:=false > /tmp/slam.log 2>&1 &
  sleep 5
  ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false params_file:=/home/malharlabade/nav2_params_200x200.yaml
)
```

---

## 3️⃣ Terminal 3: Robot State Publisher (On ALFRIDROS Pi5)

**Publish robot model for RViz visualization:**

```bash
source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=0 && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && \
ros2 run robot_state_publisher robot_state_publisher --ros-args \
-p robot_description:="$(cat ~/butler_ros2_ws/src/butler_control/urdf/butler.urdf)"
```

---

## 4️⃣ Terminal 4: RViz (On Local Machine)

**Visualize the robot:**

```bash
rviz2
```

**RViz Setup:**
1. Set **Fixed Frame** to `base_link`
2. Click **Add** and add these displays:
   - **RobotModel** → Topic: `/robot_description`
   - **Map** → Topic: `/map`
   - **LaserScan** → Topic: `/scan`, Fixed Frame: `laser`
   - **LocalCostmap** → Topic: `/local_costmap/costmap_raw`
   - **GlobalCostmap** → Topic: `/global_costmap/costmap_raw`
   - **TF** → Show all transforms

---

## ✅ Verify Everything Works

### Check all topics are publishing:

```bash
# On any machine with ROS2:
source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=0 && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && \
ros2 topic list
```

**Should show:**
- `/cmd_vel` (navigation → motors)
- `/encoder_odom` (encoders → odometry)
- `/scan` (lidar)
- `/tf`, `/tf_static` (transforms)
- `/map` (SLAM map)
- `/robot_description` (URDF)

### Test motor control:

```bash
# Terminal on any machine
source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=0 && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && \
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.3}, angular: {z: 0.0}}' --once
```

Robot should move forward slowly.

### Test navigation goal:

In RViz:
1. Click **2D Goal Pose** button
2. Click on map where you want robot to go
3. Robot should plan path and move autonomously

---

## 🎯 Motor Calibration (If robot circles)

**Run calibration test:**

```bash
python3 /home/malharlabade/motor_calibration_test.py
```

Watch robot in RViz - note which test makes it go straightest. Update encoder node:

```bash
nano ~/butler_ros2_ws/src/butler_gpio/butler_gpio/encoder_odometry_node.py
```

Change:
```python
self.left_calibration = 1.0   # Adjust if left motor was slower
self.right_calibration = 1.0  # Adjust if right motor was slower
```

Then restart encoder node (Terminal 1).

---

## 🔧 Common Adjustments

### Increase robot speed:

Edit `/home/malharlabade/nav2_params_200x200.yaml`:
```yaml
FollowPath:
  desired_linear_vel: 1.0  # from 0.5 m/s
```

### Reduce safety margin (robot closer to walls):

```yaml
inflation_layer:
  inflation_radius: 0.5  # from 0.7 meters
```

### Smaller costmap (faster planning):

```yaml
global_costmap:
  width: 100   # from 200m
  height: 100  # from 200m
```

---

## 📊 Monitoring

### Watch encoder odometry:

```bash
source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=0 && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && \
ros2 topic echo /encoder_odom --field twist.twist.linear.x
```

### Monitor motors:

```bash
source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=0 && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && \
ros2 topic echo /cmd_vel
```

### Check map quality:

```bash
source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=0 && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && \
ros2 topic echo /slam_toolbox/feedback
```

---

## 🛑 Stopping Everything

```bash
# Kill everything (or Ctrl+C in each terminal)
pkill -f "ros2"
pkill -f "python3"
```

---

## 🆘 Troubleshooting Quick Links

- **Robot circles:** See [TROUBLESHOOTING.md](TROUBLESHOOTING.md#robot-circles-when-going-straight)
- **Lidar not working:** See [TROUBLESHOOTING.md](TROUBLESHOOTING.md#rplidar-not-publishing-scan)
- **Navigation issues:** See [NAV2_CONFIG.md](NAV2_CONFIG.md)
- **URDF changes:** See [URDF.md](URDF.md)

---

## 🚀 Next Steps

1. **Test autonomous navigation** - Set goals in RViz
2. **Calibrate motors** - Run calibration test for straight movement
3. **Tune Nav2** - Adjust speeds and safety margins
4. **Build maps** - SLAM will create map as robot explores
5. **Add sensors** - Mount additional cameras, IMU, etc.

Enjoy ALFRID! 🤖

---

Property of 5KROBOTICS & MALHAR LABADE © 2026
