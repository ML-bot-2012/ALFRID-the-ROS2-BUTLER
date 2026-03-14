# ALFRID: Autonomous Edge AI Robot

**Sovereign intelligence. Real-time SLAM. Zero cloud dependencies.**

## Overview

ALFRID is a fully autonomous mobile robot that maps environments in real-time using edge computing. Built on ROS2 with a distributed system across two Raspberry Pis.
```
🤖 ALFRID ROBOT SYSTEM
├── ALFRIDCL (Pi3B+ - Ubuntu 22.04)
│   ├── Motor control (L298N driver)
│   ├── Quadrature encoders (20Hz odometry)
│   └── Real-time GPIO loop
│
└── ALFRIDROS (Pi5 - Ubuntu 24.04)
    ├── SLAM mapping (async_slam_toolbox)
    ├── RViz2 visualization (50Hz sync)
    └── Nav2 autonomous navigation
```

## Quick Start
```bash
ssh malharlabade@alfridcl.local
./alfridcli [1]  # Boot everything
./alfridcli [2]  # Start teleop
```

Control with W/A/S/D keyboard. Watch RViz as the blue robot cylinder syncs perfectly with physical movement.

## Hardware

- **RPLidar A1:** 360° scanning, 12m range
- **Encoders:** 4x quadrature, 660 CPR each
- **Motors:** 2x 12V DC with L298N driver
- **Compute:** Pi5 (8GB) + Pi3B+ (1GB)
- **Network:** WiFi + mDNS (.local hostnames)

## What It Does

1. **Explores:** Drives through environments autonomously
2. **Maps:** Builds 2D occupancy grids in real-time (SLAM)
3. **Localizes:** Tracks position via encoder + lidar fusion
4. **Visualizes:** Live RViz dashboard (robot position, obstacles, map)
5. **Navigates:** Autonomous goal-based movement (Nav2 ready)

## Demo

Boot system → Watch blue cylinder in RViz → Drive with keyboard → See real-time SLAM mapping → Set autonomous goal → Robot navigates independently.

## Hackathon Tracks

🤖 **Physical AI & Robotics** - Real robot, real sensors, real autonomy
🌐 **Sovereign Infrastructure** - Edge-only compute, zero cloud

## Repository
```
.
├── README.md (you are here)
├── QUICKSTART.md (detailed setup)
├── ARCHITECTURE.md (system design)
├── ENCODERS.md (quadrature odometry)
├── MOTORS.md (motor control)
├── SLAM.md (mapping)
├── LIDAR.md (RPLidar)
├── NAV2config.md (navigation)
├── RVIZ2.md (visualization)
├── WIRING.md (GPIO pinout)
├── URDF.md (robot model)
├── TROUBLESHOOTING.md (debugging)
└── butler_ros2_ws/ (ROS2 workspace)
```

## Key Innovation

**Quadrature Encoder Odometry:** Real-time A+B channel decoding gives us accurate position tracking without expensive IMUs.

**mDNS Networking:** alfridcl.local & alfridros.local work automatically on any WiFi. No IP hunting.

**Production Automation:** Single command `./alfridcli [1]` brings up full system in 2 minutes.

---



Property of ML-bot-2012 & Malhar Labade
