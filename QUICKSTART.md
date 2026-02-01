# BUTLER Quick Start Guide - COMPLETE

**Property of 5KROBOTICS & MALHAR LABADE**

Everything you need to get BUTLER running autonomously with all systems integrated.

---

## 📋 Prerequisites

### System Requirements
- ALFRIDCL (Pi3B+) running Ubuntu 22.04 with ROS2 Humble
- ALFRIDROS (Pi5) running Ubuntu 24.04 with ROS2 Jazzy
- Both on same network (ROS_DOMAIN_ID=0)
- 12V battery for motors (3000mAh minimum)
- USB camera (1280x960 capable)
- RPLidar A1 connected via /dev/ttyUSB0

### Network Setup
```bash
# On both Pis, add to ~/.bashrc:
export ROS_DOMAIN_ID=0

# On ALFRIDROS only, add:
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### Test Connectivity
```bash
# From any Pi:
ping ALFRIDCL
ping ALFRIDROS
# Both should respond with <50ms latency
```

---

## ⚡ Optional: AI Hardware Acceleration

### Hailo-8 13 TOPS AI HAT (HIGHLY RECOMMENDED)

Before starting main robot, optionally install GPU acceleration:

```bash
# On ALFRIDROS (Pi5):
ssh ubuntu@ALFRIDROS

# Update system
sudo apt-get update
sudo apt-get upgrade -y

# Install Hailo software
sudo apt-get install -y hailo-all

# Reboot
sudo reboot

# Verify installation
lspci | grep Hailo
# Should show: Hailo Inference Accelerator
```

**Benefits**:
- Hand detection: 30+ FPS (vs 10-15 without)
- Latency: 33ms (vs 100-150ms without)
- CPU load: 20-30% (vs 80%+ without)
- No code changes needed!

See [AI_HAT_ENHANCED.md](AI_HAT_ENHANCED.md) for full installation guide.

---

## 📡 8-Terminal Startup Sequence

### Understanding the Architecture

```
ALFRIDCL (Pi3B+ - Motor Controller)
├── Terminal 1: Motor Control (ros2 launch)
├── Terminal 2: Teleop Keyboard Control
└── Terminal 3: Debug/Monitoring (reserve)

ALFRIDROS (Pi5 - Main Computer)
├── Terminal 4: TF Transforms + RPLidar + Robot State Publisher
├── Terminal 5: SLAM Mapping (creates map in real-time)
├── Terminal 6: Nav2 Navigation (autonomous navigation)
├── Terminal 7: VNC Server (remote desktop access)
└── Terminal 8: Hand Gesture Navigator (gesture control + distance keeping)
```

### Startup Order (IMPORTANT!)

Always start in this order to avoid timing issues:

**STEP 1: ALFRIDCL Terminals (Pi3B+)**

### TERMINAL 1 (ALFRIDCL) - Motor Control

SSH into Pi3B+:
```bash
ssh malharlabade@ALFRIDCL
```

Launch motor control node:
```bash
sudo bash -c "source /opt/ros/humble/setup.bash && \
unset RMW_IMPLEMENTATION && \
export ROS_DOMAIN_ID=0 && \
source /home/malharlabade/butler_ros2_ws/install/setup.bash && \
ros2 launch butler_gpio pi3b_launch.py"
```

**Expected output:**
```
[INFO] [launch]: Default logging verbosity set to INFO
[INFO] [motor_control_node-1]: process started with pid [1234]
[motor_control_node-1] [INFO] [motor_control]: Motor Control Node initialized
```

**What it does:**
- Initializes GPIO pins on Pi3B+
- Creates ROS2 node listening on /cmd_vel topic
- Starts PWM at 1000Hz
- Auto-stops motors after 0.5s of no command

**Keep this terminal running!**

---

### TERMINAL 2 (ALFRIDCL) - Teleop Control

New SSH to Pi3B+:
```bash
ssh malharlabade@ALFRIDCL
source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=0 && python3 ~/teleop_simple.py
```

**Expected output:**
```
BUTLER Teleop Control
====================
Controls:
  W - Forward
  S - Backward
  A - Turn Left
  D - Turn Right
  SPACE - Stop
  Q - Quit
```

**Controls:**
- **W** = Forward (0.2 speed)
- **S** = Backward (-0.2 speed)
- **A** = Left Turn (0.4 angular)
- **D** = Right Turn (-0.4 angular)
- **SPACE** = Stop (0.0 speed)
- **Q** = Quit

**Test it!** Press W - robot should move forward smoothly.

---

### TERMINAL 3 (ALFRIDCL) - Reserve

Keep this open for debugging:
```bash
ssh malharlabade@ALFRIDCL
# Just connect, don't run anything yet
```

Used for:
- Checking /dev/ttyUSB0 connectivity
- Testing GPIO pins
- Monitoring system resources with `top`
- ROS2 debugging with `ros2 topic echo`

---

**STEP 2: ALFRIDROS Terminals (Pi5)**

### TERMINAL 4 (ALFRIDROS) - TF + RPLidar + RSP

SSH into Pi5:
```bash
ssh ubuntu@ALFRIDROS
```

Launch all three together:
```bash
source /opt/ros/jazzy/setup.bash && \
export ROS_DOMAIN_ID=0 && \
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && \
(nohup ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom > /tmp/tf1.log 2>&1 & \
nohup ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link > /tmp/tf2.log 2>&1 & \
nohup ros2 run tf2_ros static_transform_publisher 0.08 0 0.23 0 0 0 base_link laser > /tmp/tf3.log 2>&1 & \
sleep 1 && \
nohup ros2 run rplidar_ros rplidar_composition --ros-args -p serial_port:=/dev/ttyUSB0 -p frame_id:=laser > /tmp/rplidar.log 2>&1 & \
sleep 2 && \
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat ~/butler_ros2_ws/src/butler_control/urdf/butler.urdf)")
```

**What it does:**
1. **TF Publishers** (3 static publishers)
   - map → odom: Global to local frame
   - odom → base_link: Local to robot frame
   - base_link → laser: Robot to LiDAR frame

2. **RPLidar Driver**
   - Starts /scan topic at 5.5Hz
   - Publishes 360° 2D scans
   - Range: 0.15m - 12m

3. **Robot State Publisher**
   - Reads URDF file
   - Publishes robot structure
   - Updates transforms at ~50Hz

**Check for errors in logs:**
```bash
tail -20 /tmp/tf1.log /tmp/rplidar.log
```

---

### TERMINAL 5 (ALFRIDROS) - SLAM Mapping

New SSH to Pi5:
```bash
ssh ubuntu@ALFRIDROS
source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=0 && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && \
ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
  -p use_sim_time:=false \
  -p map_start_mode:=localization \
  -p map_start_pose:="[0.0, 0.0, 0.0]" & \
sleep 3 && \
while true; do \
  ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
    "{name: {data: '/home/ubuntu/maps/test_map'}}" > /dev/null 2>&1; \
  sleep 5; \
done
```

**What it does:**
- Starts SLAM Toolbox in **async mode** (real-time mapping)
- Creates map from LiDAR + transforms
- Auto-saves map every 5 seconds to `/home/ubuntu/maps/test_map.yaml`

**Expected output:**
```
[INFO] [slam_toolbox]: ...asynchronous_slam_toolbox_node...
[INFO] [slam_toolbox]: ...starting map with initial pose...
```

**Test it!** Drive robot around (Terminal 2 teleop) and watch map grow.

---

### TERMINAL 6 (ALFRIDROS) - Nav2 Navigation

New SSH to Pi5:
```bash
ssh ubuntu@ALFRIDROS
source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=0 && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && \
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=false \
  map:=/home/ubuntu/maps/test_map.yaml
```

**What it does:**
- Starts Navigation2 stack
- Loads saved map from Terminal 5
- Enables autonomous navigation
- Creates costmaps and path planning

**Expected output:**
```
[INFO] [launch]: All log files can be found in...
[INFO] [navigation_launch]: Starting Navigation 2 Stack...
```

**Note:** If map doesn't exist yet, this will wait. Run Terminal 5 first!

---

### TERMINAL 7 (ALFRIDROS) - VNC Server

New SSH to Pi5:
```bash
ssh ubuntu@ALFRIDROS
vncserver :1 -geometry 1024x768 -depth 24 -SecurityTypes None -localhost no --I-KNOW-THIS-IS-INSECURE
```

**What it does:**
- Starts VNC server on port 5901
- Enables remote desktop access
- Allows gesture visualization

**Connect from your Mac:**
```bash
# Option 1: Command line
open vnc://ALFRIDROS:5901

# Option 2: VNC Viewer app
# Address: ALFRIDROS:5901
# No password required
```

**Expected output:**
```
New ALPRIDROS:1 desktop created
Starting Xvfb with 1024x768 depth 24
```

---

### TERMINAL 8 (ALFRIDROS) - Hand Gesture Navigator

New SSH to Pi5:
```bash
ssh ubuntu@ALFRIDROS
DISPLAY=:1 python3 ~/ligament_distance_navigator.py
```

**What it does:**
- Detects hand gestures (open/fist)
- **With Hailo-8**: 30+ FPS, 33ms latency
- Measures hand size and calculates distance
- Publishes Twist commands to /cmd_vel
- Maintains 2-foot distance from detected hand

**Expected output:**
```
LIGAMENT DISTANCE NAVIGATOR
Hand measurements control robot movement!
Target: 2.0 feet away from robot

Distance: 2.50 ft | MOVE FORWARD
Distance: 2.05 ft | PERFECT DISTANCE!
Distance: 1.50 ft | MOVE BACKWARD
```

**Test it!** Show your hand to the camera - robot should approach/retreat.

---

## ✅ System Validation Checklist

After all 8 terminals are running, verify:

```bash
# Terminal 1 (ALFRIDCL - Motors)
☐ "Motor Control Node initialized" message
☐ No GPIO errors

# Terminal 2 (ALFRIDCL - Teleop)
☐ Keyboard input recognized
☐ W/A/S/D cause immediate motor response
☐ Movement is smooth, not jerky

# Terminal 4 (ALFRIDROS - Transforms)
☐ /scan topic has data
☐ All 3 TF publishers running
☐ No errors in /tmp/*.log files

# Terminal 5 (ALFRIDROS - SLAM)
☐ Map auto-saving to /home/ubuntu/maps/
☐ Map growing as robot moves
☐ No "lost" messages in console

# Terminal 6 (ALFRIDROS - Nav2)
☐ Nav2 fully initialized
☐ No "could not initialize map" errors
☐ Ready for navigation goals

# Terminal 7 (ALFRIDROS - VNC)
☐ Can connect from Mac with VNC Viewer
☐ Desktop visible
☐ No display lag

# Terminal 8 (ALFRIDROS - Gestures)
☐ Hand detected (FPS counter visible)
☐ Distance printed to console
☐ Robot responds to hand movements
```

---

## 🎮 Operating Procedures

### Procedure 1: Manual Control with Teleop

```bash
# Prerequisites:
# - Terminal 1: Motors running
# - Terminal 2: Teleop running

# Operation:
1. Press W to move forward
2. Press A for left turn
3. Press S to go backward
4. Press SPACE to stop
5. Press Q to exit

# Expected:
- Immediate response (<50ms)
- Smooth acceleration (no jerks)
- Stops within 0.5s of command
```

### Procedure 2: Autonomous Navigation with Gestures

```bash
# Prerequisites:
# - All 8 terminals running
# - Terminal 5: Map created and saved
# - Terminal 8: Hand detected

# Operation:
1. Show open hand to camera
2. Move hand closer/farther
3. Robot maintains 2-foot distance
4. Move hand side-to-side
5. Robot follows

# Expected:
- Smooth distance adjustment
- No jerky movements
- Maintains target distance ±0.3 feet
```

### Procedure 3: Full SLAM Exploration

```bash
# Prerequisites:
# - Terminals 1-7 running
# - Terminal 8 NOT needed for this

# Operation:
1. Terminal 2: Drive robot around room with WASD
2. Terminal 5: Monitor map creation
3. RViz: Visualize growing map
4. Terminal 6: Can now set Nav2 goals

# Expected:
- Real-time map generation
- No "lost" SLAM messages
- Map saved every 5 seconds
```

---

## 📊 Performance Benchmarks

After startup, check these metrics:

| Metric | Expected | Max | Issue if |
|--------|----------|-----|----------|
| Motor response | <50ms | 100ms | Delayed keyboard input |
| Hand detection FPS | 10-15 | <5 | Upgrade to Hailo-8 |
| SLAM update rate | Real-time | >1s lag | Reduce map resolution |
| VNC latency | <100ms | >500ms | Reduce screen res in VNC |
| ROS topic latency | <50ms | >200ms | Check network (ping) |
| CPU usage ALFRIDROS | 40-60% | >90% | Enable Hailo-8 |

---

## 🔧 Source Code: teleop_simple.py

**Location**: `~/teleop_simple.py`

```python
#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

class TeleopSimple:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('teleop_simple')
        self.publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)
        
        self.msg = Twist()
        self.linear_x = 0.0
        self.angular_z = 0.0
        
        self.settings = termios.tcgetattr(sys.stdin)
        
        print("BUTLER Teleop Control")
        print("====================")
        print("Controls:")
        print("  W - Forward")
        print("  S - Backward")
        print("  A - Turn Left")
        print("  D - Turn Right")
        print("  SPACE - Stop")
        print("  Q - Quit")
        print("")
    
    def get_key(self):
        try:
            if select.select([sys.stdin], [], [], 0.1)[0]:
                return sys.stdin.read(1).lower()
        except:
            pass
        return None
    
    def send_command(self):
        self.msg.linear.x = self.linear_x
        self.msg.angular.z = self.angular_z
        self.publisher.publish(self.msg)
    
    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key()
                
                if key == 'w':
                    self.linear_x = 1.0
                    self.angular_z = 0.0
                    print("→ FORWARD")
                
                elif key == 's':
                    self.linear_x = -1.0
                    self.angular_z = 0.0
                    print("→ BACKWARD")
                
                elif key == 'a':
                    self.linear_x = 0.0
                    self.angular_z = 1.0
                    print("→ LEFT TURN")
                
                elif key == 'd':
                    self.linear_x = 0.0
                    self.angular_z = -1.0
                    print("→ RIGHT TURN")
                
                elif key == ' ':
                    self.linear_x = 0.0
                    self.angular_z = 0.0
                    print("→ STOP")
                
                elif key == 'q':
                    print("Quitting...")
                    break
                
                self.send_command()
                rclpy.spin_once(self.node, timeout_sec=0.01)
        
        except Exception as e:
            print(f"Error: {e}")
        
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            self.msg.linear.x = 0.0
            self.msg.angular.z = 0.0
            self.publisher.publish(self.msg)
            self.node.destroy_node()
            rclpy.shutdown()

def main():
    teleop = TeleopSimple()
    teleop.run()

if __name__ == '__main__':
    main()
```

---

## 🚀 Next Steps

After successful startup:

1. **Troubleshooting?** → See [TROUBLESHOOTING_ENHANCED.md](TROUBLESHOOTING_ENHANCED.md)
2. **Motor tuning?** → See [MOTORS_ENHANCED.md](MOTORS_ENHANCED.md)
3. **Gesture customization?** → See [HAND_GESTURE_ENHANCED.md](HAND_GESTURE_ENHANCED.md)
4. **Want AI speed?** → See [AI_HAT_ENHANCED.md](AI_HAT_ENHANCED.md)
5. **Full system deep dive?** → See [README_ENHANCED.md](README_ENHANCED.md)

---

**Last Updated**: January 31, 2026  
**Version**: 2.0 (January 2026 Production Release)
