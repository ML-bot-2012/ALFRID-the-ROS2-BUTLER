# ALFRID Troubleshooting Guide

## Navigation & Localization Issues

### Robot circles when going straight

**Problem:** Robot veers left or right instead of moving forward.

**Root Cause:** Motor speed imbalance (left/right motors running at different speeds).

**Solution:**

1. **Run motor calibration test:**
   ```bash
   python3 /home/malharlabade/motor_calibration_test.py
   ```

2. **Watch robot in RViz:**
   - Test 1: Both equal → baseline
   - Test 2: LEFT faster (1.05x) → if circles RIGHT, left motor is slower
   - Test 3: RIGHT faster (1.05x) → if circles LEFT, right motor is slower

3. **Update calibration in encoder_odometry_node.py:**
   ```python
   self.left_calibration = 1.05   # If left was slower
   self.right_calibration = 1.0
   ```

4. **Restart encoder node and test again**

---

### "Robot is out of bounds of the costmap!" warning

**Problem:** Nav2 warns robot position is outside costmap bounds.

**Root Cause:** Default 100×100m costmap is too small or robot isn't localized.

**Solution:**

**Option 1 - Set initial pose in RViz:**
1. Open RViz2
2. Click "2D Pose Estimate" button
3. Click on map where robot actually is
4. Robot position syncs with costmap

**Option 2 - Use 200×200m costmap:**
```bash
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=false \
  params_file:=/home/malharlabade/nav2_params_200x200.yaml
```

---

### Map shows "No Map Received"

**Problem:** Nav2 shows empty gray map with no obstacles.

**Root Cause:** SLAM hasn't built map yet or is initializing.

**Solution:**

1. **Wait 10-15 seconds** for SLAM to initialize
2. **Check SLAM is running:**
   ```bash
   ros2 topic list | grep slam
   ```
   Should show: `/slam_toolbox/feedback`, `/slam_toolbox/scan_visualization`

3. **Check if map is publishing:**
   ```bash
   ros2 topic echo /map --once
   ```
   Should show grid data

4. **If no map:**
   - Check lidar is publishing `/scan`
   - Check odometry is publishing `/encoder_odom`
   - Restart SLAM: `pkill -f slam_toolbox`

---

### Poor localization (AMCL tracking off)

**Problem:** Robot position drifts away from actual position.

**Root Cause:** Odometry drift or poor lidar data.

**Solution:**

1. **Check encoder odometry accuracy:**
   ```bash
   ros2 topic echo /encoder_odom
   ```
   - Move robot forward 1m in straight line
   - Check if odometry reports ~1m displacement

2. **If odometry drifts, re-calibrate motors** (see "Robot circles" solution)

3. **Increase AMCL particles** in nav2_params.yaml:
   ```yaml
   amcl:
     max_particles: 3000  # from 2000
   ```

4. **Check lidar mounting:** RPLidar must be horizontal and facing outward

---

## Hardware & Communication Issues

### Motor doesn't respond / Robot won't move

**Problem:** Motor control node running but motors don't spin.

**Root Cause:** GPIO pins misconfigured or motors not powered.

**Solution:**

1. **Check motor node is running on ALFRIDCL (Pi3B+):**
   ```bash
   ssh malharlabade@192.168.86.226
   ros2 node list | grep motor
   ```

2. **Test motors directly:**
   ```bash
   python3 ~/forward.py    # Should move forward
   python3 ~/stop.py       # Should stop
   ```

3. **Check GPIO pins:**
   ```bash
   gpio readall  # on Pi3B+
   ```

4. **Verify power:**
   - Check Pi3B+ power supply
   - Check motor power connections
   - Use multimeter on motor power pins

---

### Encoder odometry not publishing

**Problem:** `/encoder_odom` topic doesn't exist.

**Root Cause:** Encoder node crashed or not started.

**Solution:**

1. **Check if running:**
   ```bash
   ros2 topic list | grep encoder
   ```

2. **Restart encoder node on ALFRIDCL:**
   ```bash
   ssh malharlabade@192.168.86.226
   pkill -f encoder_odometry
   python3 ~/butler_ros2_ws/src/butler_gpio/butler_gpio/encoder_odometry_node.py
   ```

3. **Check for GPIO errors:**
   - Verify encoder GPIO pins: RF(11/12), LR(4/8), LF(7/10), RR(15/14)
   - Test with: `gpio readall`

4. **If still failing:** Encoders may be disconnected (test with multimeter)

---

### RPLidar not publishing /scan

**Problem:** `/scan` topic missing or empty.

**Root Cause:** RPLidar not connected or on wrong USB port.

**Solution:**

1. **Check USB connection:**
   ```bash
   ls -la /dev/ttyUSB*
   ```
   Look for `/dev/ttyUSB0` or `/dev/ttyUSB1`

2. **Find which port lidar is on:**
   ```bash
   dmesg | grep ttyUSB
   ```

3. **Update launch file with correct port:**
   ```python
   parameters=[{'serial_port': '/dev/ttyUSB0'}]  # Change if needed
   ```

4. **Check lidar physically:**
   - Verify USB cable is connected
   - Check RPLidar spinning (should see rotation)
   - Listen for motor sound (faint buzzing)

5. **Restart RPLidar:**
   ```bash
   pkill -f rplidar
   ros2 run rplidar_ros rplidar_composition --ros-args -p serial_port:=/dev/ttyUSB0
   ```

---

### DDS Communication Errors (ROS2 Humble ↔ Jazzy)

**Problem:** Warnings about "Failed to parse type hash" in logs.

**Root Cause:** Jazzy and Humble use different DDS implementations.

**Solution:**

These are harmless warnings. To reduce them:

1. **Use Cyclone DDS consistently:**
   ```bash
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   ```

2. **Ensure both Pi5 and Pi3B+ set this variable** before starting nodes

3. **These warnings are safe to ignore** - data still transmits correctly

---

## Network & ROS2 Issues

### Nodes can't find each other (discovery issues)

**Problem:** Pi5 nodes can't see Pi3B+ nodes or vice versa.

**Root Cause:** Different ROS_DOMAIN_ID or network misconfiguration.

**Solution:**

1. **Verify ROS_DOMAIN_ID is consistent:**
   ```bash
   echo $ROS_DOMAIN_ID  # Should be 0 on both
   ```

2. **Set on both Pi5 AND Pi3B+:**
   ```bash
   export ROS_DOMAIN_ID=0
   ```

3. **Check network connectivity:**
   ```bash
   ping 192.168.86.226  # From Pi5 to Pi3B+
   ping 192.168.86.222  # From Pi3B+ to Pi5
   ```

4. **Restart all nodes** after fixing environment variables

---

### Transforms missing (No transform from X to Y)

**Problem:** RViz shows "No transform" errors in red.

**Root Cause:** Static transforms not publishing.

**Solution:**

1. **Check transforms are running:**
   ```bash
   ros2 topic echo /tf_static --once
   ```

2. **Required transforms for ALFRID:**
   - `map → odom`
   - `odom → base_link`
   - `base_link → laser`
   - `base_link → left_wheel`, `right_wheel`, `caster_wheel`

3. **If missing, restart static transforms:**
   ```bash
   # Terminal 1
   ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom &
   ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link &
   ```

4. **Verify transforms exist:**
   ```bash
   ros2 topic list | grep tf
   ```
   Should show `/tf` and `/tf_static`

---

## RViz Display Issues

### Robot doesn't appear (blue box missing)

**Problem:** RViz shows empty map, no robot visualization.

**Root Cause:** RobotModel display not configured or URDF not loading.

**Solution:**

1. **Verify URDF is loading:**
   ```bash
   ros2 param get /robot_state_publisher robot_description
   ```
   Should show XML content, not empty

2. **In RViz:**
   - Set Fixed Frame to `base_link`
   - Add Display → RobotModel
   - Set Topic to `/robot_description`
   - Robot should appear

3. **If still missing:**
   - Check robot_state_publisher is running
   - Verify `/robot_description` topic exists
   - Look for errors in robot_state_publisher terminal

---

### Lidar points not visible

**Problem:** `/scan` topic exists but no red dots in RViz.

**Root Cause:** LaserScan display not configured properly.

**Solution:**

1. **Add LaserScan display in RViz:**
   - Click "Add" button
   - Select "LaserScan"
   - Set Topic: `/scan`
   - Set Fixed Frame: `laser`

2. **Verify scan data:**
   ```bash
   ros2 topic echo /scan --once | head -20
   ```
   Should show angle_min, angle_max, ranges array

3. **Check laser frame exists:**
   ```bash
   ros2 topic echo /tf_static | grep laser
   ```

---

## Terminal Command Issues

### "Command not found" when running scripts

**Problem:** `python3 forward.py` fails with command not found.

**Solution:**

1. **Use full paths:**
   ```bash
   python3 /home/malharlabade/forward.py
   ```

2. **Or change directory first:**
   ```bash
   cd /home/malharlabade && python3 forward.py
   ```

3. **Or make executable:**
   ```bash
   chmod +x ~/forward.py
   ~/forward.py
   ```

---

### SSH connection issues

**Problem:** `ssh malharlabade@192.168.86.226` times out.

**Solution:**

1. **Check network:**
   ```bash
   ping 192.168.86.226
   ```

2. **Verify correct IP (might have changed):**
   ```bash
   ssh malharlabade@192.168.86.222  # Try Pi5 first
   ```

3. **Check SSH service on target:**
   ```bash
   sudo systemctl status ssh
   ```

4. **Use different approach - run on same Pi:**
   ```bash
   # Instead of SSHing, log in directly to Pi
   ```

---

## Performance Issues

### High CPU usage / System slow

**Problem:** Pi5 or Pi3B+ running hot, processes maxed out.

**Root Cause:** Too many nodes, SLAM expensive, costmap resolution too fine.

**Solution:**

1. **Check CPU usage:**
   ```bash
   top -b -n 1 | head -20
   ```

2. **Reduce costmap resolution:**
   ```yaml
   resolution: 0.1  # from 0.05 (2x faster)
   ```

3. **Reduce AMCL particles:**
   ```yaml
   max_particles: 1000  # from 2000
   ```

4. **Lower control frequency:**
   ```yaml
   controller_frequency: 10.0  # from 20.0
   ```

5. **Kill unused nodes:**
   ```bash
   pkill -f slam_toolbox      # if not needed
   pkill -f nav2              # if not needed
   ```

---

## Getting Help

If issue persists:

1. **Collect logs:**
   ```bash
   ros2 doctor --report > diagnosis.txt
   ```

2. **Check node status:**
   ```bash
   ros2 node list
   ros2 node info /motor_control
   ```

3. **View recent errors:**
   ```bash
   ~/.ros/log/latest/
   ```

4. **Record rosbag for analysis:**
   ```bash
   ros2 bag record -a  # Records all topics
   ```

---

Property of 5KROBOTICS & MALHAR LABADE © 2026
