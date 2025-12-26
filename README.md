# ALFRID - Autonomous SLAM Robot Butler BY 5KROBOTICS and MALHAR LABADE

ALFRID is an autonomous SLAM robot butler inspired by Batman's faithful servant. 

## Overview
Built on ROS2 with dual Raspberry Pi architecture, ALFRID autonomously explores and maps environments in real-time using advanced SLAM algorithms.

## Hardware
- **Main Computer**: Raspberry Pi 5 (Ubuntu 22.04 Jammy)
- **GPIO Controller**: Raspberry Pi 3B+ (Ubuntu 22.04 Jammy)
- **Sensors**: RPLidar A1, 4-channel encoder sensors
- **Motors**: 2x DIY 12V DC Encoder Gear Motors and 1 Micro Servo(Metal Gears)
- **Power**: 12V battery → L298N Motor Driver
- **Custom PCB** for integrated control

## Software Stack
- **Pi5**: ROS2 Jazzy Jalisco - SLAM Toolbox, Nav2, real-time mapping
- **Pi3B+**: ROS2 Humble Hawksbill - motor control, encoder odometry
- **Communication**: WiFi networked via ROS2 Domain ID 0

## Key Features
✅ Real-time SLAM mapping with RPLidar A1
✅ Dual-encoder motor control for navigation
✅ Autonomous exploration and environment mapping
✅ Distributed ROS2 architecture (Pi5 + Pi3B+)
✅ Custom PCB integration

## Installation
```bash
pip3 install alfrid-slam  # Pi5
pip3 install alfrid-gpio  # Pi3B+
```

## Quick Start
**Pi5**: `ros2 launch slam_toolbox online_async_launch.py`
**Pi3B+**: `ros2 launch butler_gpio pi3b_launch.py`

---
Property of 5KROBOTICS & MALHAR LABADE
