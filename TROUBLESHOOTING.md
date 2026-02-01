# BUTLER Troubleshooting Guide

Complete troubleshooting for all BUTLER systems.

---

## 🔴 Motor Issues

### Motors Not Moving

**Symptoms**: Terminal 1 running but no motor response

**Solutions**:
```bash
# 1. Check GPIO pins are accessible
python3 << 'EOF'
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(25, GPIO.OUT)  # Left forward
GPIO.setup(23, GPIO.OUT)  # Right forward
print("✅ GPIO pins accessible")
GPIO.cleanup()
EOF

# 2. Check 12V battery voltage
# Use multimeter on battery terminals
# Should read 12V (minimum 11V)

# 3. Verify motor driver (L298N) connections
# Check all wires are firmly connected

# 4. Test with direct PWM:
python3 << 'EOF'
import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
GPIO.setup(6, GPIO.OUT)
pwm = GPIO.PWM(6, 1000)
pwm.start(100)  # 100% speed
time.sleep(2)
pwm.stop()
GPIO.cleanup()
print("✅ PWM working")
EOF
```

### Motors Moving in Wrong Direction

**Solutions**:
```bash
# Swap forward/backward pins in motor_control_node.py
# LEFT MOTOR:
#   self.left_forward = 25  → swap with
#   self.left_backward = 5

# Then redeploy:
cd ~/butler_ros2_ws
colcon build --packages-select butler_gpio
source install/setup.bash
ros2 launch butler_gpio pi3b_launch.py
```

### Jerky/Stuttering Movement

**Causes**: Usually CPU load or power supply

**Solutions**:
```bash
# 1. Check CPU usage
top
# Should be <50% for motor control alone

# 2. Check voltage stability
# Add capacitor across 12V battery (220µF or larger)

# 3. Reduce network traffic on ALFRIDCL
# Terminal 3 should be mostly idle

# 4. Enable Hailo-8 on ALFRIDROS
# This frees CPU significantly
```

---

## 📸 Camera & Gesture Issues

### Camera Not Detected

**Symptoms**: OpenCV returns None or false for cap.isOpened()

**Solutions**:
```bash
# 1. List USB devices
lsusb
# Should show "Sonix Technology Co."

# 2. Check device permissions
ls -la /dev/video*
# You should have at least /dev/video0

# 3. Test with v4l2-ctl
apt-get install v4l2-utils
v4l2-ctl --list-devices

# 4. Check if capture works
python3 << 'EOF'
import cv2
cap = cv2.VideoCapture(0)
ret, frame = cap.read()
print(f"Camera works: {ret}, Shape: {frame.shape if ret else 'N/A'}")
cap.release()
EOF
```

### Hand Not Detected (Gesture Recognition Issues)

**Symptoms**: No hand detected even with hand in view

**Solutions**:
```bash
# 1. Check lighting
# Minimum 200 lux recommended
# Check for backlighting washing out hand

# 2. Adjust detection thresholds in code:
# In ligament.py, change:
self.hands = self.mp_hands.Hands(
    min_detection_confidence=0.5,  # Increase from 0.3
    min_tracking_confidence=0.5    # Increase from 0.3
)

# 3. Test with ligament.py directly:
DISPLAY=:1 python3 ~/ligament.py
# Hold hand at 3 feet, open palm
# Should print hand measurements

# 4. Check if Hailo-8 is interfering
# Temporarily disable and test on CPU
```

### Low FPS / Laggy Gesture Detection

**Symptoms**: <15 FPS even in ligament.py

**Solutions**:
```bash
# 1. Install Hailo-8 (solves this for most users)
sudo apt-get install -y hailo-all

# 2. Reduce resolution:
self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# 3. Reduce detection frequency:
# Only process every 2nd frame

# 4. Check CPU temp
vcgencmd measure_temp
# If >80°C, add heatsink to Pi5
```

### Distance Navigator Not Moving Robot

**Symptoms**: Hand detected but robot doesn't respond

**Solutions**:
```bash
# 1. Check motors are running (Terminal 1)
# Look for: "Motor Control Node initialized"

# 2. Verify ROS_DOMAIN_ID matches
echo $ROS_DOMAIN_ID
# Must be 0 on both Pis

# 3. Test teleop manually
# Terminal 2: Press W key
# Robot should move forward

# 4. Check /cmd_vel is being published:
ros2 topic echo /cmd_vel
# Should see twist messages when hand is detected
```

---

## 🗺️ SLAM & Navigation Issues

### SLAM Not Creating Map

**Symptoms**: Slam_toolbox running but no map_server output

**Solutions**:
```bash
# 1. Check LiDAR connection
ls -la /dev/ttyUSB0
# Should exist and be readable

# 2. Verify RPLidar is running
ros2 topic echo /scan
# Should see continuous scan data

# 3. Check transforms
ros2 run tf2_tools view_frames.py
# Should show: map → odom → base_link → laser

# 4. Verify SLAM output directory exists
mkdir -p /home/ubuntu/maps
chmod 777 /home/ubuntu/maps

# 5. Check SLAM output in Terminal 5:
# Should show: "[AsyncSlamToolbox] Starting mapper..."
```

### Map Not Saving

**Symptoms**: /home/ubuntu/maps/ empty

**Solutions**:
```bash
# 1. Manually save map from command line:
ros2 service call /slam_toolbox/save_map \
  slam_toolbox/srv/SaveMap \
  "{name: {data: '/home/ubuntu/maps/test_map'}}"

# 2. Check permissions
ls -la /home/ubuntu/maps/
chmod 755 /home/ubuntu/maps

# 3. Verify SLAM service exists:
ros2 service list | grep slam
# Should include /slam_toolbox/save_map

# 4. Manual backup in Terminal 5:
while true; do 
  ros2 service call /slam_toolbox/save_map \
    slam_toolbox/srv/SaveMap \
    "{name: {data: '/home/ubuntu/maps/backup_$(date +%s)'}}"
  sleep 10
done
```

### Nav2 Not Initializing

**Symptoms**: "No map loaded" or nav2 crashes

**Solutions**:
```bash
# 1. Verify map file exists:
ls -la /home/ubuntu/maps/test_map.*
# Should have .yaml and .pgm files

# 2. Check map format:
# Map must be from SLAM Toolbox, not other sources

# 3. Try manual startup:
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=false \
  map:=/home/ubuntu/maps/test_map.yaml

# 4. Check Nav2 logs:
tail -100 ~/.ros/log/*/*.log | grep nav2
```

---

## 🔌 Network & ROS Issues

### Pis Can't Communicate

**Symptoms**: ros2 commands timeout, "Could not contact the middleware"

**Solutions**:
```bash
# 1. Check both Pis on same network:
ping ALFRIDROS
ping ALFRIDCL
# Both should respond with <50ms latency

# 2. Verify ROS_DOMAIN_ID:
echo $ROS_DOMAIN_ID
# Must be 0 on both Pis (set in .bashrc)

# 3. Check RMW on ALFRIDROS only:
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# ALFRIDCL should use default rmw_connext

# 4. Test discovery:
# Terminal on ALFRIDROS:
ros2 node list
# Should show nodes from ALFRIDCL

# 5. Restart ROS if stuck:
pkill -f ros2
pkill -f python3
# Wait 5 seconds, restart terminals
```

### Teleop Commands Delayed

**Symptoms**: 1-2 second delay between key press and motor response

**Solutions**:
```bash
# 1. Check network latency:
ping -c 5 ALFRIDROS
# Should be <20ms average

# 2. Reduce CycloneDDS overhead on ALFRIDROS
# Add to /etc/cyclonedds/dds_security_conf.xml:
<NetworkInterface name="auto"/>  # Single interface only

# 3. Increase motor timeout:
# In motor_control_node.py:
self.cmd_timeout = 1.0  # Increase from 0.5s

# 4. Check wireless signal
iwconfig
# Should show good signal strength
```

---

## 🎛️ VNC Remote Desktop Issues

### Can't Connect to VNC

**Symptoms**: VNC client timeout or refused

**Solutions**:
```bash
# 1. Check VNC is running:
ssh ubuntu@ALFRIDROS
ps aux | grep vncserver
# Should show running vncserver

# 2. Restart VNC:
vncserver -kill :1
sleep 2
vncserver :1 -geometry 1024x768 -depth 24 \
  -SecurityTypes None -localhost no

# 3. Check port 5901 is open:
ss -tlnp | grep 5901
# Should show LISTENING

# 4. Connect with correct address:
VNC Viewer → ALFRIDROS:5901
# NOT 192.168.86.222:5901

# 5. If still no image:
# Make sure DISPLAY=:1 is set before launching apps
export DISPLAY=:1
python3 ~/ligament.py
```

### VNC Display Frozen

**Symptoms**: VNC connects but display doesn't update

**Solutions**:
```bash
# 1. Restart X server:
ssh ubuntu@ALFRIDROS
killall Xvfb  # Or equivalent
# VNC will auto-restart

# 2. Restart specific app:
# In Terminal 8:
Ctrl+C  # Stop ligament_distance_navigator.py
sleep 2
DISPLAY=:1 python3 ~/ligament_distance_navigator.py

# 3. Check DISPLAY variable:
echo $DISPLAY
# Should be :1 (not empty or :0)
```

---

## 🔋 Power Issues

### Brownout Resets (System Crashes)

**Symptoms**: Random reboots, especially during motor movement

**Solutions**:
```bash
# 1. Check power supply capacity:
# 12V battery should be 3000mAh minimum
# Voltmeter test: Should not drop below 11.5V under load

# 2. Add capacitors:
# 220µF or larger across motor power
# 100µF across 5V Pi power

# 3. Check USB power
# Pi 5 needs 5V/5A minimum
# Terminal shows if under-voltage:
grep Under /var/log/syslog

# 4. Reduce concurrent tasks:
# Don't run gesture + SLAM + Nav2 simultaneously if power-limited

# 5. Monitor voltage:
while true; do
  vcgencmd measure_volts core
  vcgencmd measure_volts sdram_c
  sleep 1
done
```

### Battery Drains Too Fast

**Symptoms**: Only 1-2 hours runtime

**Solutions**:
```bash
# 1. Check CPU usage:
top -b -n1 | head -20
# If >70% average, disable unnecessary processes

# 2. Reduce camera FPS:
cap.set(cv2.CAP_PROP_FPS, 15)  # Reduce from 30

# 3. Disable VNC if not needed:
vncserver -kill :1

# 4. Reduce SLAM processing:
# Only save map every 30 seconds instead of 5

# 5. Lower Wi-Fi power:
iwconfig
# Use 802.11n (2.4GHz) instead of 802.11ac
```

---

## 📊 Testing Procedures

### Full System Test

```bash
# 1. Terminal 1 - Motors (5 min test)
# Watch for: GPIO init, no errors

# 2. Terminal 2 - Teleop (press all keys)
# Watch for: Immediate response, smooth movement

# 3. Terminal 7 - VNC
# Watch for: Connection successful, no display lag

# 4. Terminal 8 - Gestures (5 min)
# Watch for: FPS counter, hand detection, colored borders

# 5. Stress test (30 min):
# Run all terminals simultaneously
# Monitor: Temperatures, CPU, no crashes
```

### Checkpoint Validation

```bash
# Before deployment, verify:
☐ Motors respond to all commands
☐ Hand detected at 10 feet
☐ Distance navigator maintains 2-foot target
☐ SLAM creates readable map
☐ Can set Nav2 goals
☐ Hailo-8 showing 30+ FPS (if installed)
☐ No crashes in 30-min stress test
☐ Runtime >4 hours on full battery
```

---

**Last Updated**: January 31, 2026  
**Version**: 2.0 (January 2026 Production Release)
