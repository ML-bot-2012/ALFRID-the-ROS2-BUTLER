# ALFRID - Autonomous SLAM Robot Butler

**Property of 5KROBOTICS & MALHAR LABADE**

ALFRID is an autonomous SLAM robot butler inspired by Batman's faithful servant.

```
    _      _       _____   ____    ___   ____     
   / \    | |     |  ___| |  _ \  |_ _| |  _ \    
  / _ \   | |     | |_    | |_) |  | |  | | | |   
 / ___ \  | |___  |  _|   |  _ <   | |  | |_| |   
/_/   \_\ |_____| |_|     |_|_\_\ |___| |____/_   
| |_  | |__     ___       |  _ \   / _ \  / ___|  
| __| | '_ \   / _ \      | |_) | | | | | \___ \  
| |_  | | | | |  __/      |  _ <  | |_| |  ___) | 
 \__|_|_| |_|  \___|____  |_| \_\  \___/  |____/  
  | __ )  | | | | |_   _| | |     | ____| |  _ \  
  |  _ \  | | | |   | |   | |     |  _|   | |_) | 
  | |_) | | |_| |   | |   | |___  | |___  |  _ <  
  |____/   \___/    |_|   |_____| |_____| |_| \_\
```

---

## 🎯 Overview

Built on ROS2 with dual Raspberry Pi architecture, ALFRID autonomously explores and maps environments in real-time using advanced SLAM algorithms.

---

## 🔧 Hardware

- **Main Computer**: Raspberry Pi 5 (Ubuntu 22.04 Jammy)
- **GPIO Controller**: Raspberry Pi 3B+ (Ubuntu 22.04 Jammy)
- **Sensors**: RPLidar A1, 4-channel encoder sensors
- **Motors**: 2x DIY 12V DC Encoder Gear Motors + 1 Micro Servo (Metal Gears)
- **Power**: 12V battery → L298N Motor Driver
- **Custom PCB** for integrated control *(Optional - breadboard + jumper cables work too!)*

---

## 💻 Software Stack

- **Pi5**: ROS2 Jazzy Jalisco - SLAM Toolbox, Nav2, real-time mapping
- **Pi3B+**: ROS2 Humble Hawksbill - motor control, encoder odometry
- **Communication**: WiFi networked via ROS2 Domain ID 0

---

## ✨ Key Features

✅ Real-time SLAM mapping with RPLidar A1  
✅ Dual-encoder motor control for navigation  
✅ Autonomous exploration and environment mapping  
✅ Distributed ROS2 architecture (Pi5 + Pi3B+)  
✅ Custom PCB integration (optional)  

---

## 📥 Installation Guide

### ⚠️ CRITICAL: Before Starting

You'll need **proper markdown image display**. Here's why the original images didn't show:

**Problem**: Original README used HTML `<img>` tags which most markdown viewers don't render
**Solution**: This README uses proper markdown syntax `![alt](url)` which displays correctly

### Step 1: Download Ubuntu 22.04 Image

**Visit**: [Ubuntu Jammy Releases](https://cdimage.ubuntu.com/ubuntu/releases/jammy/release/)

**File to download**: `ubuntu-22.04.5-preinstalled-server-arm64+raspi.img.xz`

Look for the download button in the file list.

```bash
# Alternative: Download via terminal
cd ~/Downloads
wget https://cdimage.ubuntu.com/ubuntu/releases/jammy/releases/ubuntu-22.04.5-preinstalled-server-arm64+raspi.img.xz
```

**File size**: ~2GB
**Download time**: 5-10 minutes (depends on internet)

---

### Step 2: Download Raspberry Pi Imager

**Visit**: [Raspberry Pi Imager Official](https://www.raspberrypi.com/software/)

**Choose your OS**:
- 🍎 **macOS**: Download `.dmg` file → Open → Drag to Applications
- 🪟 **Windows**: Download `.exe` file → Run installer
- 🐧 **Linux**: Use package manager

```bash
# macOS (with Homebrew):
brew install raspberry-pi-imager
open -a "Raspberry Pi Imager"

# Ubuntu/Debian:
sudo apt-get install rpi-imager

# Fedora:
sudo dnf install rpi-imager
```

---

### Step 3: Write Image to Pi3B+ SD Card

1. **Insert MicroSD card** into reader on your Mac/PC
2. **Open Raspberry Pi Imager**
3. **Click "Choose device"** → Select **Raspberry Pi 3B+**
4. **Click "Choose OS"** → Scroll to bottom → Select **"Custom"**
5. **Browse** to the `ubuntu-22.04.5-preinstalled-server-arm64+raspi.img.xz` file you downloaded
6. **Click "Choose storage"** → Select your MicroSD card
7. **Review the summary** (should show Pi3B+ and Ubuntu 22.04)
8. **Click "Next"** → Skip customization options
9. **Click "Yes"** to confirm write (may ask for password)
10. ⏳ **Wait 10-15 minutes** for write to complete
11. ✅ **"Writing completed successfully"** → Eject MicroSD card

---

### Step 4: Write Image to Pi5 SD Card

1. **Remove first MicroSD card** from reader
2. **Insert second MicroSD card** into reader
3. **Open Raspberry Pi Imager** again
4. **Click "Choose device"** → Select **Raspberry Pi 5**
5. **Repeat steps 4-11 from above**
6. ✅ Both SD cards now ready!

---

### Step 5: Insert SD Cards into Raspberry Pis

**For Pi3B+**:
```
MicroSD Slot Location: Bottom of Pi3B+
Action: Push card in until it clicks (fully inserted)
```

**For Pi5**:
```
MicroSD Slot Location: Bottom edge of Pi5
Action: Push card in until it clicks (fully inserted)
NOTE: Pi5 slot is slightly different - don't force!
```

---

### Step 6: Connect Power & Displays

**Pi3B+ Setup**:
- HDMI cable → Monitor (standard HDMI)
- USB mouse → Pi3B+ USB port
- USB keyboard → Pi3B+ USB port
- USB power (5V/2.5A) → Pi3B+ Micro USB port

**Pi5 Setup**:
- Micro HDMI cable → Monitor (⚠️ NOT standard HDMI!)
- USB mouse → Pi5 USB-A port
- USB keyboard → Pi5 USB-A port
- USB-C power (5V/5A) → Pi5 USB-C port

⏳ System boots automatically - wait 1-2 minutes for Ubuntu logo

---

### Step 7: Configure Hostname

When you see the login prompt:

```bash
# Default credentials:
# Username: ubuntu
# Password: ubuntu
# (will ask to change on first login)

# Set hostname for Pi3B+ (Motor Controller):
sudo hostnamectl set-hostname ALFRIDCL
hostnamectl  # Verify shows: ALFRIDCL

# Set hostname for Pi5 (Main Computer):
sudo hostnamectl set-hostname ALFRIDROS
hostnamectl  # Verify shows: ALFRIDROS
```

**This is CRITICAL** - Both Pis must have correct hostnames for ROS2 networking!

---

### Step 8: Connect to Wi-Fi

```bash
# Scan available networks:
nmcli device wifi list

# Connect to your Wi-Fi:
nmcli device wifi connect "YOUR_WIFI_NAME" password "YOUR_PASSWORD"

# Verify connection:
ping google.com
# Should show successful pings

# Note your IP address:
hostname -I
# Shows something like: 192.168.1.100
```

---

### Step 9: Install ROS2 on Both Systems

**On BOTH Pi3B+ and Pi5**:

```bash
# Update system packages:
sudo apt-get update
sudo apt-get upgrade -y

# Install ROS2 Humble:
sudo apt-get install -y ros-humble-ros-core

# Add to bash profile:
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verify installation:
ros2 --version
# Should show: ROS 2 humble
```

**On Pi5 ONLY (additional)**:

```bash
# Install ROS2 Jazzy (newer version):
sudo apt-get install -y ros-jazzy-ros-core

# Add to bash profile:
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# Install networking middleware (required):
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc

# Install CycloneDDS package:
sudo apt-get install -y ros-jazzy-rmw-cyclonedds-cpp
```

---

### Step 10: Configure ROS2 Domain ID

**On BOTH Pi3B+ and Pi5**:

```bash
# Edit bash configuration:
nano ~/.bashrc

# Scroll to end and add:
export ROS_DOMAIN_ID=0
export COLCON_DEFAULTS_YAML=~/.colcon/defaults.yaml

# Save: Press Ctrl+X, then Y, then Enter

# Apply changes:
source ~/.bashrc

# Verify:
echo $ROS_DOMAIN_ID
# Should print: 0
```

**Why?** ROS_DOMAIN_ID allows the two Pis to find each other on the network!

---

### Step 11: Install ALFRID Software Packages

```bash
# On BOTH Pi3B+ and Pi5:

cd ~/Downloads

# Download ALFRID packages from GitHub:
wget https://github.com/ML-bot-2012/ALFRID-the-ROS2-BUTLER/releases/download/butler/alfrid_slam-1.0.0-py3-none-any.whl
wget https://github.com/ML-bot-2012/ALFRID-the-ROS2-BUTLER/releases/download/butler/alfrid_gpio-1.0.0-py3-none-any.whl

# Install the packages:
pip3 install alfrid_slam-1.0.0-py3-none-any.whl
pip3 install alfrid_gpio-1.0.0-py3-none-any.whl

# Verify installation:
pip3 list | grep alfrid
# Should show both packages
```

**If downloads fail**: See [WHY_IMAGES_NOT_DISPLAYING.md](WHY_IMAGES_NOT_DISPLAYING.md) for alternatives

---

### Step 12: Setup Tiger VNC (Remote Access)

```bash
# On BOTH Pi3B+ and Pi5:

# Install VNC and desktop:
sudo apt-get install -y tigervnc-server xfce4 xfce4-goodies

# Start VNC server:
vncserver :1 -geometry 1024x768 -depth 24

# When prompted:
# Enter new VNC password (write it down!)
# Confirm password
# View-only password? → No

# Kill and restart (to apply settings):
vncserver -kill :1
vncserver :1 -geometry 1024x768 -depth 24
```

**From your Mac/PC**:
```
1. Download VNC Viewer from realvnc.com
2. Open VNC Viewer
3. Connect to: ALFRIDCL:5901 (for Pi3B+)
            or ALFRIDROS:5901 (for Pi5)
4. Enter VNC password you created
5. You should see the desktop!
```

---

## 📚 Complete Documentation

For more detailed information, see:

| File | Purpose |
|------|---------|
| [INSTALLATION_STEPS.md](INSTALLATION_STEPS.md) | Detailed setup with troubleshooting |
| [WHY_IMAGES_NOT_DISPLAYING.md](WHY_IMAGES_NOT_DISPLAYING.md) | Image display issues explained |
| [QUICKSTART_ENHANCED.md](QUICKSTART_ENHANCED.md) | How to run all 8 terminals |
| [MOTORS_ENHANCED.md](MOTORS_ENHANCED.md) | Motor control system |
| [HAND_GESTURE_ENHANCED.md](HAND_GESTURE_ENHANCED.md) | Camera & gesture recognition |
| [LIGAMENT_NAVIGATOR_ENHANCED.md](LIGAMENT_NAVIGATOR_ENHANCED.md) | Distance-keeping autonomy |
| [AI_HAT_ENHANCED.md](AI_HAT_ENHANCED.md) | GPU acceleration with Hailo-8 |
| [TROUBLESHOOTING_ENHANCED.md](TROUBLESHOOTING_ENHANCED.md) | Fix common problems |

---

## ✅ Verify Setup Works

After installation, test that everything is connected:

```bash
# Test system connectivity:
ping ALFRIDROS  # From Pi3B+
ping ALFRIDCL   # From Pi5

# Both should respond with <50ms latency

# Test ROS2 communication:
# Terminal 1 on Pi5:
ros2 topic pub /test std_msgs/String "data: 'Hello from Pi5'"

# Terminal 2 on Pi3B+:
ros2 topic echo /test
# You should see the message appear!
```

---

## 🚀 Next Steps

1. ✅ Follow this **README_FIXED.md** to install
2. ✅ Go to **[INSTALLATION_STEPS.md](INSTALLATION STEPS.md)** for detailed help
3. ✅ Go to **[QUICKSTART_ENHANCED.md](QUICKSTART.md)** to run ALFRID
4. ✅ Check **[TROUBLESHOOTING_ENHANCED.md](TROUBLESHOOTING.md)** if issues arise

---

## 📞 Support & Issues

**GitHub Repository**: [ALFRID-the-ROS2-BUTLER](https://github.com/ML-bot-2012/ALFRID-the-ROS2-BUTLER)

**Report Issues**:
- Check [TROUBLESHOOTING_ENHANCED.md](TROUBLESHOOTING_ENHANCED.md) first
- Go to GitHub Issues tab
- Include: Error message + what step failed + your hardware

---

**Property of 5KROBOTICS & MALHAR LABADE**

*Last Updated: January 31, 2026*
*Version: 2.0 (January 2026 Production Release)*
