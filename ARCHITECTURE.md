# ALFRID Robot System Architecture

## System Overview

ALFRID is a **distributed ROS2 system** running on two Raspberry Pi computers connected via Ethernet, communicating through DDS (Distributed Data Service).

```
┌─────────────────────────────────────────────────────────────────┐
│                        ALFRID Robot (5KROBOTICS)                │
├─────────────────────────────────────────────────────────────────┤
│                                                                   │
│  ┌──────────────────────────┐          ┌────────────────────┐  │
│  │    ALFRIDROS (Pi5)       │          │   ALFRIDCL (Pi3B+) │  │
│  │  192.168.86.222          │◄────────►│ 192.168.86.226     │  │
│  │  Ubuntu 24.04            │  Ethernet│ Ubuntu 22.04       │  │
│  │  ROS2 Jazzy              │          │ ROS2 Humble        │  │
│  │                          │          │                    │  │
│  │  ✓ SLAM Toolbox          │          │ ✓ GPIO Control     │  │
│  │  ✓ Nav2 Stack            │          │ ✓ Motor Control    │  │
│  │  ✓ RPLidar Driver        │          │ ✓ Encoder Reading  │  │
│  │  ✓ Robot State Publisher │          │ ✓ 4x Encoders      │  │
│  │                          │          │ ✓ DDS Bridge       │  │
│  └──────────────────────────┘          └────────────────────┘  │
│                                                                   │
└─────────────────────────────────────────────────────────────────┘
          ▲                                      ▲
          │                                      │
      RViz2 (Local Machine)               Physical Robot
      192.168.86.X:5900                   (Sensors & Motors)
```

---

## Hardware Components

### ALFRIDROS (Raspberry Pi 5)
**Role:** SLAM, Navigation, Planning, Perception

| Component | Specs |
|-----------|-------|
| **CPU** | Raspberry Pi 5 (2.4 GHz, 8 cores) |
| **RAM** | 8 GB |
| **OS** | Ubuntu 24.04 LTS |
| **ROS2** | Jazzy Jalisco |
| **Network** | Gigabit Ethernet, 192.168.86.222 |
| **DDS** | Cyclone DDS |

**Running:**
- SLAM Toolbox (async_slam_toolbox_node)
- Nav2 (navigation_launch.py)
- RPLidar A1 Driver (rplidar_composition)
- Robot State Publisher (URDF visualization)
- TF2 Transform Publisher
- ROS2 Bridge to ALFRIDCL

---

### ALFRIDCL (Raspberry Pi 3B+)
**Role:** Real-time GPIO Control, Motor Commands, Encoder Feedback

| Component | Specs |
|-----------|-------|
| **CPU** | Raspberry Pi 3B+ (1.4 GHz, 4 cores) |
| **RAM** | 1 GB |
| **OS** | Ubuntu 22.04 LTS |
| **ROS2** | Humble Hawksbill |
| **Network** | Gigabit Ethernet, 192.168.86.226 |
| **GPIO** | 27 pins available |
| **PWM** | 2 channels (motors) |
| **DDS** | Default (bridges to Cyclone) |

**Running:**
- Motor Control Node (PWM to motors)
- Encoder Odometry Node (quadrature decoding)
- GPIO Manager (limit switches, servo)
- ROS2 Bridge to ALFRIDROS

---

### Sensors

| Sensor | Connection | Data |
|--------|-----------|------|
| **RPLidar A1** | `/dev/ttyUSB0` (USB serial) | `/scan` (LaserScan) |
| **4x Encoders** | GPIO (quadrature decoding) | `/encoder_odom` (Odometry) |
| **Motor PWM** | GPIO (Pins 5,25,6,23,24,22) | From `/cmd_vel` |
| **Limit Switches** | GPIO (17, 27) | Collision detection |

---

## Software Architecture

### Data Flow: Navigation

```
RViz (Goal) ──► Nav2 (navigation_launch.py)
                  ├─► Global Planner (NavFn Dijkstra)
                  ├─► Local Planner (MPPI Controller)
                  ├─► Costmap Manager (200×200m)
                  └─► /cmd_vel Publisher

            /cmd_vel ──► Motor Control (ALFRIDCL)
                  ├─► PWM Generator
                  └─► Motors Spin

            Motor Spinning ──► Encoders (quadrature)
                  └─► /encoder_odom Publisher

            /encoder_odom + /scan ──► SLAM Toolbox
                  └─► /map Publisher

            /map + Robot Pose ──► RViz Visualization
```

### Data Flow: DDS Communication

```
ALFRIDROS (Jazzy)  ◄─────── DDS Network ─────────► ALFRIDCL (Humble)
├─ /cmd_vel          (→ Motor Control)
├─ /scan             (← RPLidar)
├─ /map              (← SLAM)
├─ /tf, /tf_static   (← All Transforms)
├─ /robot_description (← URDF)
├─ /encoder_odom     (← Encoders)
└─ /joint_states     (← Robot State)
```

---

## ROS2 Nodes Map

### On ALFRIDROS (Pi5 - Jazzy)

```
rplidar_composition
  ├─ Publishes: /scan (LaserScan)
  └─ Device: /dev/ttyUSB0

slam_toolbox::async_slam_toolbox_node
  ├─ Subscribes: /scan, /encoder_odom
  ├─ Publishes: /map, /tf, /slam_toolbox/*
  └─ Role: Build map from lidar + odometry

robot_state_publisher::robot_state_publisher
  ├─ Parameter: robot_description (URDF)
  ├─ Publishes: /tf_static, /joint_states
  └─ Role: Publish robot model

nav2_bringup::navigation_launch.py
  ├─ controller_server (MPPI Controller)
  ├─ smoother_server
  ├─ planner_server (NavFn Planner)
  ├─ behavior_server
  ├─ bt_navigator (Behavior Trees)
  ├─ velocity_smoother
  ├─ collision_monitor
  └─ lifecycle_manager

tf2_ros::static_transform_publisher
  ├─ Publishes: map ─► odom ─► base_link
  ├─           base_link ─► left_front_wheel
  ├─           base_link ─► right_front_wheel
  ├─           base_link ─► left_rear_wheel
  ├─           base_link ─► right_rear_wheel
  ├─           base_link ─► laser
  └─ Role: Fixed transform tree
```

### On ALFRIDCL (Pi3B+ - Humble)

```
motor_control_node (custom Python)
  ├─ Subscribes: /cmd_vel (Twist)
  ├─ GPIO Output: Motor PWM (pins 5,25,6,23,24,22)
  ├─ GPIO Output: Motor Direction (pins 5,25,23,24)
  └─ Role: Convert velocity to motor commands

encoder_odometry_node (custom Python)
  ├─ GPIO Input: Encoder quadrature (8 pins)
  ├─ Publishes: /encoder_odom (Odometry)
  ├─ Publishes: /tf (odom ─► base_link)
  └─ Role: Track robot position from wheel rotation
```

---

## Communication Protocol: DDS

### ROS2 DDS Middleware

**ALFRIDROS:** Cyclone DDS (optimized for embedded)
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

**ALFRIDCL:** Default DDS (rmw_fastrtps)
```bash
# No explicit export needed (falls back to default)
```

**Why it works:**
- Both implement standard DDS RTPS protocol
- Cyclone DDS is compatible with other DDS implementations
- No explicit bridges needed (DDS discovers participants automatically)

### ROS_DOMAIN_ID

```bash
export ROS_DOMAIN_ID=0  # Both Pi5 and Pi3B+ use domain 0
```

All nodes on same domain share a DDS virtual network.

### DDS Topics Published

| Topic | Type | Hz | Source |
|-------|------|-----|--------|
| `/scan` | sensor_msgs/LaserScan | 10 | RPLidar (Pi5) |
| `/encoder_odom` | nav_msgs/Odometry | 20 | Encoders (Pi3B+) |
| `/cmd_vel` | geometry_msgs/Twist | 20 | Nav2 (Pi5) |
| `/map` | nav_msgs/OccupancyGrid | 1 | SLAM (Pi5) |
| `/tf` | tf2_msgs/TFMessage | 50 | Various |
| `/tf_static` | tf2_msgs/TFMessage | 1 | Static Publisher (Pi5) |

---

## Motor Control System

### Hardware Setup (ALFRIDCL Pi3B+)

**Left Motor (GPIO 5, 25, 6):**
- GPIO 5: Forward Enable
- GPIO 25: Backward Enable  
- GPIO 6: PWM (0-100%)

**Right Motor (GPIO 23, 24, 22):**
- GPIO 23: Forward Enable
- GPIO 24: Backward Enable
- GPIO 22: PWM (0-100%)

### Motor Command Flow

```
/cmd_vel (Twist msg)
  linear.x = 0.5 m/s
  angular.z = 0.2 rad/s
       ↓
motor_control_node (Differential Drive)
  left_speed = linear - angular = 0.3 m/s
  right_speed = linear + angular = 0.7 m/s
       ↓
PWM Generator
  Left PWM:  30% duty cycle
  Right PWM: 70% duty cycle
       ↓
Motors Spin
  Left slower, Right faster → Robot turns right
```

### Motor Speed Calibration

**Issue:** Motors run at different speeds → robot circles

**Solution:** Speed calibration factors in encoder_odometry_node.py

```python
self.left_calibration = 1.0   # Multiply left distance by this
self.right_calibration = 1.0  # Multiply right distance by this
```

If left motor is 5% slower:
```python
self.left_calibration = 1.05  # Compensate by 5%
```

---

## Encoder Odometry System

### 4x Encoders (660 CPR each)

| Encoder | GPIO A | GPIO B | Wheel |
|---------|--------|--------|-------|
| RF (Right Front) | 12 | 11 | Right |
| LR (Left Rear) | 4 | 8 | Left |
| LF (Left Front) | 10 | 7 | Left |
| RR (Right Rear) | 15 | 14 | Right |

### Odometry Calculation

```
Encoder counts → Motor rotation
  660 counts = 1 full rotation

Motor rotation → Wheel distance
  Wheel diameter = 0.065m
  Distance = π × 0.065 × (counts / 660)

Left + Right distance → Robot motion
  Forward: (left + right) / 2
  Turn: (right - left) / wheelbase
```

### Odometry Frame

Published as Odometry message:
```
/encoder_odom
├─ header.frame_id: "odom"
├─ child_frame_id: "base_link"
├─ pose.pose.position: (x, y, z)
├─ pose.pose.orientation: (quaternion)
└─ twist.twist: (linear, angular velocity)
```

---

## SLAM & Mapping (SLAM Toolbox)

### Inputs Required

| Input | Topic | Source | Rate |
|-------|-------|--------|------|
| Lidar Scans | `/scan` | RPLidar | 10 Hz |
| Odometry | `/encoder_odom` | Encoders | 20 Hz |
| Robot TF | `/tf` | Static Publisher | 50 Hz |

### Outputs Produced

| Output | Topic | Rate | Use |
|--------|-------|------|-----|
| Map | `/map` | 1 Hz | Nav2 planning |
| Map → Odom TF | `/tf` | 10 Hz | Localization |
| Scan Viz | `/slam_toolbox/scan_visualization` | 10 Hz | RViz debug |
| Graph | `/slam_toolbox/graph_visualization` | 1 Hz | Map structure |

### How It Works

```
Lidar Scan (t=0)
    ↓
SLAM Matcher: Find where robot is in existing map
    ├─ Use encoder odometry as initial guess
    ├─ Scan matching to refine position
    └─ Update particle filter (AMCL)
    ↓
Update Map Locally
    ├─ Add new scan points to map
    ├─ Mark new obstacles
    └─ Clear old free space
    ↓
Loop Closure Detection (optional)
    ├─ Did we see this place before?
    ├─ If yes, constrain map poses
    └─ Reduce drift over time
    ↓
Publish /tf (map ─► odom)
```

---

## Navigation Stack (Nav2)

### Pipeline

```
User Goal ──► NavFn Planner ──► Path
  (RViz)     (Dijkstra on        (sequence of
             global costmap)     waypoints)
                  ↓
         MPPI Controller ──► Velocity Command
         (local planning)    (/cmd_vel Twist)
                  ↓
         Motor Control ──► Robot Moves
         (ALFRIDCL)
                  ↓
         Collision Monitor ──► Emergency Stop
         (checks ahead)
```

### Costmap Layers

```
Global Costmap (200m × 200m)
├─ Static Layer: Walls from /map
├─ Obstacle Layer: Dynamic obstacles from /scan
└─ Inflation Layer: Safety buffer around obstacles

Local Costmap (3m × 3m rolling window)
├─ Obstacle Layer: Immediate obstacles
└─ Inflation Layer: Close-range safety
```

---

**Wheel Configuration (4-wheel Differential Drive):**

```
Left Front Wheel (LF)      Right Front Wheel (RF)
        ●                           ●
        
        ╔═══════════════════════════╗
        ║   ALFRID CHASSIS          ║
        ║   (Base Link)             ║
        ╚═══════════════════════════╝
        
        ●                           ●
Left Rear Wheel (LR)       Right Rear Wheel (RR)

All 4 wheels: 0.0325m radius, 660 CPR encoders
Motor control: 2 DC motors via L298N driver
```

---

## Network Topology

```
┌─────────────────────────────────────────┐
│         Home Network (192.168.86.0/24)  │
│                                         │
│  ┌──────────────┐       ┌────────────┐  │
│  │ ALFRIDROS    │       │ ALFRIDCL   │  │
│  │ 192.168.86.222       │192.168.86.226 │
│  │ (Jazzy)      │       │ (Humble)   │  │
│  └──────┬───────┘       └────────┬───┘  │
│         │                        │       │
│         └────────┬───────────────┘       │
│                  │                       │
│           Gigabit Ethernet               │
│           (DDS Discovery)                │
│                  │                       │
│  ┌──────────────┴────────────────┐      │
│  │                               │      │
│  ▼                               ▼      │
│ RViz2 (Local Machine)      (Local ssh)  │
│ 192.168.86.X               Terminal     │
└─────────────────────────────────────────┘
```

---

## System Startup Sequence

```
1. Power on both Pi5 and Pi3B+
   ↓
2. ALFRIDCL (Pi3B+): Start motor & encoder nodes
   ├─ motor_control_node ready to receive /cmd_vel
   └─ encoder_odometry_node publishing /encoder_odom
   ↓
3. ALFRIDROS (Pi5): Start static transforms (2 seconds)
   ├─ map ─► odom ─► base_link transforms
   └─ Wheels, laser frame transforms
   ↓
4. ALFRIDROS: Start RPLidar (1 second)
   └─ Publishing /scan at 10 Hz
   ↓
5. ALFRIDROS: Start SLAM Toolbox (2 seconds after RPLidar)
   ├─ Subscribe: /scan, /encoder_odom
   ├─ Wait for initial matching
   └─ Start publishing /map, /tf
   ↓
6. ALFRIDROS: Start Nav2 (after SLAM ready, ~10 seconds)
   ├─ Load costmap from /map
   ├─ Initialize planner & controller
   └─ Ready to accept navigation goals
   ↓
7. Start RViz on local machine
   ├─ Set Fixed Frame to base_link
   ├─ Add displays for map, robot, lidar
   └─ Ready for autonomous navigation
```

**Total startup time:** ~15-20 seconds

---

## Performance Characteristics

| Metric | Value | Notes |
|--------|-------|-------|
| **Navigation Latency** | 100-200 ms | Goal to motor command |
| **SLAM Update Rate** | 10 Hz | New map/pose estimates |
| **Odometry Rate** | 20 Hz | Encoder feedback |
| **Control Frequency** | 20 Hz | Nav2 controller updates |
| **Costmap Resolution** | 0.05 m | 5cm per cell |
| **Max Speed** | 0.5 m/s | Configurable (safety limit) |
| **Max Turning** | 3.2 rad/s² | Angular acceleration limit |
| **CPU Usage (Pi5)** | 30-40% | SLAM + Nav2 active |
| **CPU Usage (Pi3B+)** | 5-10% | Motor control + encoder |
| **Network Bandwidth** | ~2 Mbps | DDS traffic |

---

## System Constraints & Limitations

### Hardware Limitations
- **Pi3B+ GPIO:** Only 27 pins (all in use)
- **Pi5 USB:** 4 ports (RPLidar uses 1)
- **Pi3B+ Real-time:** Not true RTOS (soft real-time only)
- **Wireless:** DDS works better over Ethernet than WiFi

### Software Limitations
- **SLAM:** Requires odometry accuracy (encoder calibration critical)
- **Nav2:** Doesn't work in completely unknown spaces (needs some map)
- **Motors:** No velocity feedback (open-loop PWM control)
- **Costmap:** 200×200m at 0.05m resolution requires 2+ seconds to initialize

### Environmental Limitations
- **RPLidar A1:** 12m max range (good for indoors/medium outdoor)
- **Wheel Slip:** Affects odometry accuracy
- **Lighting:** None (no light sensors)
- **Elevation:** Not equipped for stairs/slopes

---

## Future Improvements

1. **Add IMU** - Improve odometry with gyroscope
2. **Motor Encoder Feedback** - Direct speed control instead of PWM
3. **Camera** - Visual odometry, person following
4. **Bumper Sensor** - Physical collision detection
5. **Battery Management** - Monitor voltage, auto shutdown
6. **Wireless Mode** - WiFi instead of Ethernet
7. **RTC** - Real-time clock for timestamping
8. **EEPROM** - Persistent configuration storage

---

## References

- [ROS2 Documentation](https://docs.ros.org/en/jazzy/)
- [Nav2 Architecture](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [DDS Overview](https://www.dds-foundation.org/)
- [Raspberry Pi GPIO](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html)

---

Property of 5KROBOTICS & MALHAR LABADE © 2026
