# ALFRID Quick Start Guide

## Prerequisites

- ALFRIDCL (Pi3B+) with ROS2 Humble
- ALFRIDROS (Pi5) with ROS2 Jazzy
- Both on same WiFi/network
- ROS_DOMAIN_ID=0 exported in ~/.bashrc on both

## Boot System
```bash
ssh malharlabade@alfridcl.local

./alfridcli [1]
```

This executes 14 automated steps:
1. GPIO pin export
2. Kill old processes
3. Start motors
4. Start encoder odometry node (20Hz)
5. Launch Robot State Publisher
6. Launch RViz movement visualizer
7. Launch TF publishers
8. Launch RPLidar driver
9. Launch SLAM Toolbox
10. Launch VNC server
11. Launch fresh RViz2 instance
12. Launch Ligament Navigator (optional)

**Wait 2-3 minutes for full boot.**

RViz window opens on your display showing:
- 🔵 Blue cylinder = robot body position
- 🔴 Red arrow = heading direction
- 🟢 Green dots = 360° lidar points
- ⚪ White/gray areas = SLAM map building live

## Teleop Control

In a new terminal:
```bash
./alfridcli [2] TELEOP
```

Keyboard commands:
- **W** = Move forward
- **A** = Turn left
- **S** = Move backward
- **D** = Turn right
- **SPACE** = Stop
- **Q** = Quit

**Watch RViz** as you drive. Blue cylinder moves in real-time, perfectly synced at 50Hz.

## Save Map
```bash
./alfridcli [4] SAVE MAP
```

Saves map as .pgm + .yaml files (Nav2 compatible).

## Autonomous Navigation
```bash
./alfridcli [5] NAV2
```

Set goal position in RViz using "2D Nav Goal" button. Robot navigates autonomously.

## Stop Motor
```bash
./alfridcli [6] STOP
```

Motors stop but system keeps running.

## Kill Everything
```bash
./alfridcli [7] KILL ALL
```

Stops all nodes. System offline.

## Menu

Instead of [1], [2], etc., just run:
```bash
./alfridcli
```

Interactive menu appears:
```
[1] START SLAM+RPLIDAR
[2] TELEOP
[4] SAVE MAP
[5] NAV2
[6] STOP
[7] KILL ALL
[0] EXIT
```

Pick option and it runs.

## Troubleshooting

**RViz won't open:**
```bash
./alfridcli [11]  # Relaunch RViz fresh
```

**Encoders not reading:**
```bash
./alfridcli [3]  # Restart encoder node
```

**SLAM not building map:**
```bash
./alfridcli [9]  # Restart SLAM
```

**WiFi lost:**
mDNS automatically reconnects. If stuck:
```bash
sudo systemctl restart wpa_supplicant
```

---
