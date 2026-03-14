# ALFRID System Architecture

## Dual Raspberry Pi Design
```
┌─────────────────────────────────────────────┐
│           ALFRID Robot System               │
├─────────────────────────────────────────────┤
│                                              │
│  ALFRIDCL (Pi3B+)      ALFRIDROS (Pi5)      │
│  Ubuntu 22.04          Ubuntu 24.04         │
│  ROS2 Humble           ROS2 Jazzy           │
│                                              │
│  ┌──────────────────┐  ┌──────────────────┐ │
│  │  GPIO Control    │  │  SLAM Mapping    │ │
│  │  Motor PWM       │  │  RViz Viz        │ │
│  │  Encoder Read    │  │  Nav2 Planning   │ │
│  │  Real-time Loop  │  │  Map Building    │ │
│  └──────────────────┘  └──────────────────┘ │
│           │                    │             │
│           └────── ROS2 DDS ────┘             │
│           (ROS_DOMAIN_ID=0)                 │
│           (mDNS hostnames)                  │
│                                              │
└─────────────────────────────────────────────┘
         ▲                       ▲
         │                       │
    Physical Sensors         RViz Display
    (Motors, Encoders)       (Real-time viz)
```

## Network Configuration

**Hostnames:**
- alfridcl.local (Pi3B+)
- alfridros.local (Pi5)

**ROS Domain ID:** 0 (shared across both Pis)

**Middleware:** FastRTPS/CycloneDDS

**Connection:** WiFi (mDNS) or ethernet

## Software Stack

### ALFRIDCL (Pi3B+ - Motor Control)

**Nodes:**
- `motor_control_node` - PWM GPIO driving
- `encoder_odometry_node` - Quadrature decoding (20Hz)
- `robot_state_publisher` - URDF loading
- ROS2 Humble

**Topics Published:**
- `/odom` - Position + orientation (20Hz)
- `/tf` - Transform chain (20Hz)

**Topics Subscribed:**
- `/cmd_vel` - Motor commands (from teleop)

### ALFRIDROS (Pi5 - Perception & Navigation)

**Nodes:**
- `async_slam_toolbox` - Real-time mapping
- `rplidar_node` - Lidar scanning (30Hz)
- `rviz2` - Visualization
- `nav2_bringup` - Navigation stack
- ROS2 Jazzy

**Topics Published:**
- `/map` - Occupancy grid (50Hz)
- `/scan` - Lidar points (30Hz)

**Topics Subscribed:**
- `/odom` - From Pi3B+ encoder
- `/cmd_vel` - Goal/velocity commands

## Data Flow
```
Physical Movement
    │
    ▼
Encoder Counting (GPIO reads)
    │
    ▼
encoder_odometry_node (Pi3B+)
Publishes: /odom, /tf at 20Hz
    │
    │
    ▼
ALFRIDROS (Pi5) receives via ROS_DOMAIN_ID=0
    │
    ├─→ SLAM Toolbox + RPLidar scan
    │   Produces: /map (50Hz)
    │
    └─→ RViz visualization
        Displays: Robot pos + heading + lidar + map
        Updates: 50Hz sync
```

## Key Features

**Real-Time Synchronization:**
- Encoder feedback: 20Hz
- RViz visualization: 50Hz
- Perfect sync means what you see matches what robot does

**Self-Healing Network:**
- mDNS .local hostnames work on any WiFi
- No IP hunting required
- Both Pis find each other automatically

**Edge-Only Compute:**
- SLAM runs locally on Pi5
- No cloud calls
- Complete data sovereignty

**Production Ready:**
- Single command boot (`./alfridcli [1]`)
- Automatic process management
- Lifecycle-managed ROS2 nodes

---
