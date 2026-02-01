# ALFRID Installation Steps - Complete Setup Guide

**Property of 5KROBOTICS & MALHAR LABADE**

Complete step-by-step installation guide for ALFRID ROS2 Butler robot with dual Raspberry Pi architecture.

---

## 🎯 Prerequisites

### What You'll Need

**Hardware:**
- Raspberry Pi 5 (8GB recommended)
- Raspberry Pi 3B+ (1GB)
- 2x MicroSD cards (32GB minimum each)
- MicroSD card reader
- HDMI cable for Pi5 (Micro HDMI)
- HDMI cable for Pi3B+ (Standard HDMI)
- Monitor, keyboard, mouse
- Wi-Fi network access
- 12V battery pack (3000mAh minimum)
- USB camera (1280x960 capable)
- RPLidar A1 with USB connection

**Software (on your Mac/PC):**
- Raspberry Pi Imager (download from raspberrypi.com)
- Terminal/Command Prompt
- Text editor (VS Code, nano, etc.)

---

## 📥 Step 1: Download Ubuntu 22.04 Image

### Option A: Download Pre-configured Image

```bash
# On your Mac/PC:
cd ~/Downloads

# Download Ubuntu 22.04 Jammy for Raspberry Pi
# Visit: https://cdimage.ubuntu.com/ubuntu/releases/jammy/release/

# For Pi5: Download "ubuntu-22.04.x-preinstalled-server-arm64+raspi.img.xz"
# For Pi3B+: Download same image (works on both)

# File will be named something like:
# ubuntu-22.04.5-preinstalled-server-arm64+raspi.img.xz

# Or download via terminal:
wget https://cdimage.ubuntu.com/ubuntu/releases/jammy/releases/ubuntu-22.04.5-preinstalled-server-arm64+raspi.img.xz
```

### Option B: Use Alternative Images

If Ubuntu 22.04 is unavailable:

```bash
# Ubuntu 24.04 (newer, also works)
wget https://cdimage.ubuntu.com/ubuntu/releases/noble/release/ubuntu-24.04-preinstalled-server-arm64+raspi.img.xz

# Or use Raspberry Pi OS (Debian-based)
# Visit: https://www.raspberrypi.com/software/operating-systems/
```

---

## 🖼️ Step 2: Install Raspberry Pi Imager

### Mac

```bash
# Download from: https://www.raspberrypi.com/software/
# Or install via Homebrew:
brew install raspberry-pi-imager

# Launch:
open -a "Raspberry Pi Imager"
```

### Windows/Linux

```bash
# Download from: https://www.raspberrypi.com/software/
# Or install via package manager:

# Ubuntu/Debian:
sudo apt-get install rpi-imager

# Fedora:
sudo dnf install rpi-imager

# Windows: Download installer from website above
```

---

## 💾 Step 3: Write Images to MicroSD Cards

### For Pi3B+ (Motor Controller)

```bash
# 1. Insert first MicroSD card into reader
# 2. Open Raspberry Pi Imager

# In Imager:
# 1. Click "Choose device" → Raspberry Pi 3B+
# 2. Click "Choose OS" → Scroll down → "Custom"
# 3. Select ubuntu-22.04.x-preinstalled-server-arm64+raspi.img.xz
# 4. Click "Choose storage" → Select your MicroSD card
# 5. Review summary (should show Pi3B+ and Ubuntu 22.04)
# 6. Click "Next" → Skip customization
# 7. Click "Yes" to write (enter password if prompted)

# Wait 10-15 minutes for write to complete

# When done: "Writing completed successfully"
# Eject MicroSD card
```

### For Pi5 (Main Computer)

```bash
# 1. Remove first MicroSD card
# 2. Insert second MicroSD card into reader
# 3. Open Raspberry Pi Imager again (or use existing window)

# In Imager:
# 1. Click "Choose device" → Raspberry Pi 5
# 2. Click "Choose OS" → Scroll down → "Custom"
# 3. Select same ubuntu-22.04.x-preinstalled-server-arm64+raspi.img.xz
# 4. Click "Choose storage" → Select second MicroSD card
# 5. Review summary (should show Pi5 and Ubuntu 22.04)
# 6. Click "Next" → Skip customization
# 7. Click "Yes" to write (enter password if prompted)

# Wait 10-15 minutes for write to complete

# When done: "Writing completed successfully"
# Eject MicroSD card
```

---

## 🔌 Step 4: Insert SD Cards into Raspberry Pis

### Pi3B+ Setup

```bash
# 1. Power off Pi3B+ (if running)
# 2. Locate MicroSD card slot on bottom of Pi3B+
# 3. Push MicroSD card in until it clicks (fully inserted)
# 4. Connect HDMI cable (regular HDMI)
# 5. Connect mouse and keyboard
# 6. Connect USB power cable

# Pi3B+ will boot automatically
# First boot takes 1-2 minutes
```

### Pi5 Setup

```bash
# 1. Power off Pi5 (if running)
# 2. Locate MicroSD card slot on bottom edge of Pi5
# 3. Push MicroSD card in until it clicks (fully inserted)
# 4. Connect Micro HDMI cable (Pi5 uses Micro HDMI, not standard HDMI)
# 5. Connect mouse and keyboard
# 6. Connect USB-C power cable (5V/5A minimum)

# Pi5 will boot automatically
# First boot takes 1-2 minutes
# You'll see Ubuntu logo and boot messages
```

---

## ⚙️ Step 5: Initial System Configuration

### Both Pi3B+ and Pi5 (On each Raspberry Pi)

When you see the login prompt or initial setup:

```bash
# 1. Default credentials:
# Username: ubuntu
# Password: ubuntu

# You'll be prompted to change password on first login
# Enter new strong password (write it down!)

# 2. System will update automatically
# Wait for this to complete (5-10 minutes)

# 3. At the prompt, configure hostname:
# This is CRITICAL for ROS2 communication!

# For Pi3B+ (Motor Controller):
sudo hostnamectl set-hostname ALFRIDCL
# Confirm with: hostnamectl

# For Pi5 (Main Computer):
sudo hostnamectl set-hostname ALFRIDROS
# Confirm with: hostnamectl
```

### Network Setup

```bash
# After hostname is set, configure Wi-Fi:

# 1. Scan available networks:
nmcli device wifi list

# 2. Connect to your Wi-Fi:
nmcli device wifi connect "YOUR_WIFI_NAME" password "YOUR_PASSWORD"

# 3. Verify connection:
ping google.com
# Should show responses

# 4. Note your IP address:
hostname -I
# Shows something like: 192.168.1.100
```

---

## 📦 Step 6: Install ROS2 on Both Systems

### On Both Pi3B+ (ALFRIDCL) and Pi5 (ALFRIDROS)

```bash
# 1. Update system packages:
sudo apt-get update
sudo apt-get upgrade -y

# 2. Install ROS2 minimal:
sudo apt-get install -y ros-humble-ros-core

# 3. Source setup script:
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 4. Verify installation:
ros2 --version
# Should show: ROS 2 humble...
```

### Additional: On Pi5 Only (ALFRIDROS)

```bash
# Pi5 runs ROS2 Jazzy (newer distribution)
sudo apt-get install -y ros-jazzy-ros-core

# Add Jazzy to bashrc:
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# Set CycloneDDS middleware (required for network):
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc

# Install CycloneDDS:
sudo apt-get install -y ros-jazzy-rmw-cyclonedds-cpp
```

---

## 🔗 Step 7: Configure ROS2 Domain ID

### Critical: Both Systems Must Match

This allows the two Pis to communicate via ROS2.

**On BOTH Pi3B+ and Pi5:**

```bash
# 1. Edit bashrc:
nano ~/.bashrc

# 2. Add these lines at the end:
export ROS_DOMAIN_ID=0
export COLCON_DEFAULTS_YAML=~/.colcon/defaults.yaml

# 3. Save file (Ctrl+X, Y, Enter)

# 4. Apply changes:
source ~/.bashrc

# 5. Verify:
echo $ROS_DOMAIN_ID
# Should print: 0
```

---

## 📥 Step 8: Install ALFRID Packages

### On Both Pi3B+ and Pi5

```bash
# 1. Create downloads directory:
mkdir -p ~/Downloads

# 2. Download ALFRID packages:
cd ~/Downloads

# Get the latest release:
wget https://github.com/ML-bot-2012/ALFRID-the-ROS2-BUTLER/releases/download/butler/alfrid_slam-1.0.0-py3-none-any.whl
wget https://github.com/ML-bot-2012/ALFRID-the-ROS2-BUTLER/releases/download/butler/alfrid_gpio-1.0.0-py3-none-any.whl

# 3. Install packages:
pip3 install alfrid_slam-1.0.0-py3-none-any.whl
pip3 install alfrid_gpio-1.0.0-py3-none-any.whl

# 4. Verify installation:
pip3 list | grep alfrid
# Should show both packages

# 5. Build packages:
cd ~/butler_ros2_ws
colcon build
source install/setup.bash
```

### If Download Fails

```bash
# Try alternative download method:
git clone https://github.com/ML-bot-2012/ALFRID-the-ROS2-BUTLER.git
cd ALFRID-the-ROS2-BUTLER
colcon build
source install/setup.bash

# Or install dependencies manually:
sudo apt-get install -y \
  ros-humble-slam-toolbox \
  ros-humble-nav2-bringup \
  ros-humble-rplidar-ros \
  python3-colcon-common-extensions
```

---

## 🎛️ Step 9: Configure Hardware Connections

### Pi3B+ (ALFRIDCL) - Motor Controller

```bash
# 1. Connect L298N Motor Driver:
#    GND → Pi Ground (Pin 6)
#    +5V → Pi 5V (Pin 2)
#    IN1 → GPIO 25
#    IN2 → GPIO 5
#    IN3 → GPIO 23
#    IN4 → GPIO 24
#    ENA → GPIO 6 (PWM)
#    ENB → GPIO 22 (PWM)
#    OUT1,2 → LEFT motor
#    OUT3,4 → RIGHT motor

# 2. Verify GPIO access:
python3 << 'EOF'
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
print("✅ GPIO access successful")
GPIO.cleanup()
EOF

# 3. Test GPIO pins:
gpio readall
# Should show all pin states
```

### Pi5 (ALFRIDROS) - Sensors

```bash
# 1. Connect RPLidar A1:
#    Power: 12V from battery
#    GND: Ground
#    RX → USB adapter (TXD)
#    TX → USB adapter (RXD)
#    USB adapter to Pi5 USB port

# 2. Verify LiDAR connection:
ls -la /dev/ttyUSB*
# Should show /dev/ttyUSB0

# 3. Connect USB camera:
#    Standard USB → USB-A port on Pi5

# 4. Verify camera:
ls -la /dev/video*
# Should show /dev/video0
```

---

## 🖥️ Step 10: Setup Tiger VNC for Remote Access

### On Both Systems

```bash
# 1. Install TigerVNC server:
sudo apt-get install -y tigervnc-server

# 2. Create VNC configuration:
vncserver :1 -geometry 1024x768 -depth 24

# Follow prompts:
# - Enter new VNC password (can be different from system password)
# - Confirm password
# - Would you like to enter a view-only password? → No

# 3. Kill existing VNC server:
vncserver -kill :1

# 4. Create startup script:
cat > ~/.vnc/xstartup << 'EOF'
#!/bin/bash
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS
exec startxfce4
EOF

# 5. Make executable:
chmod +x ~/.vnc/xstartup

# 6. Install XFCE (lightweight desktop):
sudo apt-get install -y xfce4 xfce4-goodies

# 7. Restart VNC:
vncserver -kill :1
vncserver :1 -geometry 1024x768 -depth 24
```

### On Your Mac/PC - Connect via VNC

```bash
# 1. Download VNC Viewer:
# Mac: https://www.realvnc.com/en/connect/download/viewer/macos/
# Windows/Linux: https://www.realvnc.com/en/connect/download/viewer/

# 2. Open VNC Viewer

# 3. In address bar, enter:
ALFRIDCL:5901
# Or for Pi5:
ALFRIDROS:5901

# 4. Enter VNC password you created

# 5. You should now see desktop!
```

---

## 🚀 Step 11: Verify ROS2 Communication

### Test Network Connectivity

**From any system, SSH into both:**

```bash
# Test connectivity:
ping ALFRIDCL
ping ALFRIDROS

# Both should respond with <50ms latency

# SSH into Pi3B+:
ssh ubuntu@ALFRIDCL
# Enter password

# SSH into Pi5:
ssh ubuntu@ALFRIDROS
# Enter password
```

### Test ROS2 Communication

```bash
# On ALFRIDROS (Pi5), open terminal:
ssh ubuntu@ALFRIDROS

# Start a simple ROS2 publisher:
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 topic pub /test_topic std_msgs/String "data: 'Hello from Pi5'"

# In another terminal on ALFRIDCL (Pi3B+):
ssh ubuntu@ALFRIDCL

# Subscribe to see if message reaches:
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0

ros2 topic echo /test_topic

# You should see messages appearing!
```

---

## ✅ Step 12: Verification Checklist

### Before Running ALFRID

```bash
☐ Both Pi3B+ and Pi5 booted successfully
☐ Both have hostnames set (ALFRIDCL, ALFRIDROS)
☐ Both connected to Wi-Fi
☐ Both have ROS2 installed
☐ Both have ROS_DOMAIN_ID=0 set
☐ ALFRIDROS has CycloneDDS enabled
☐ ALFRID packages installed on both
☐ TigerVNC running on both
☐ Can SSH into both systems
☐ Can connect via VNC
☐ RPLidar detected on ALFRIDROS (/dev/ttyUSB0)
☐ USB camera detected on ALFRIDROS (/dev/video0)
☐ GPIO working on ALFRIDCL (gpio readall)
☐ ROS2 communication works between systems
```

---

## 🔧 Step 13: Troubleshooting Common Issues

### System Won't Boot

```bash
# Problem: Raspberry Pi doesn't start after SD card insertion

# Solutions:
# 1. Check power connections:
#    - Pi5: USB-C 5V/5A power
#    - Pi3B+: Micro USB 5V/2.5A power

# 2. Try different HDMI cable (sometimes faulty)

# 3. Rewrite SD card if boot hangs

# 4. Check LED indicators:
#    - Red LED: Power
#    - Green LED: Activity (should blink on startup)

# If still won't boot:
# - Use Raspberry Pi Imager to write SD card again
# - Try official Raspberry Pi OS first (to verify hardware)
```

### Can't SSH or Ping Systems

```bash
# Problem: Can't reach ALFRIDCL or ALFRIDROS

# Solutions:
# 1. Verify both on same Wi-Fi network:
ssh ubuntu@ALFRIDCL
nmcli device wifi list
# Check if your Wi-Fi appears

# 2. Reconnect to Wi-Fi:
nmcli device wifi connect "YOUR_WIFI" password "PASSWORD"

# 3. Check hostname:
hostnamectl
# Should show ALFRIDCL or ALFRIDROS

# 4. Try IP address directly:
ping 192.168.1.100  # (replace with actual IP)
ssh ubuntu@192.168.1.100

# 5. Check mDNS is working:
# Sometimes ".local" names don't resolve
# Use IP address instead
```

### ROS2 Communication Not Working

```bash
# Problem: ros2 topic pub/echo doesn't show messages

# Solutions:
# 1. Verify domain ID on both:
echo $ROS_DOMAIN_ID
# Must be 0 on both

# 2. Verify CycloneDDS on Pi5:
echo $RMW_IMPLEMENTATION
# Should be: rmw_cyclonedds_cpp

# 3. Check ROS2 installation:
ros2 --version

# 4. Source setup scripts in each terminal:
source /opt/ros/humble/setup.bash  # Pi3B+
source /opt/ros/jazzy/setup.bash   # Pi5

# 5. Try direct IP instead of hostname:
export ROS_LOCALHOST_ONLY=0
# Then retry communication

# 6. Check firewall:
sudo ufw status
# If active, allow ROS2 ports:
sudo ufw allow 11311/udp
sudo ufw allow 11312/udp
```

### VNC Connection Fails

```bash
# Problem: Can't connect via VNC viewer

# Solutions:
# 1. Verify VNC is running:
ps aux | grep vnc
# Should show vncserver process

# 2. Check VNC port:
lsof -i :5901
# Should show vnc server listening

# 3. Restart VNC:
vncserver -kill :1
vncserver :1 -geometry 1024x768 -depth 24

# 4. Verify firewall allows VNC:
sudo ufw allow 5901/tcp

# 5. Try using IP address instead of hostname:
192.168.1.100:5901

# 6. Check VNC logs:
tail -50 ~/.vnc/*.log
```

---

## 📚 Next Steps

After successful installation:

1. **Test Motors**: See [MOTORS_ENHANCED.md](MOTORS_ENHANCED.md)
2. **Test Cameras**: See [HAND_GESTURE_ENHANCED.md](HAND_GESTURE_ENHANCED.md)  
3. **Run Full System**: See [QUICKSTART_ENHANCED.md](QUICKSTART_ENHANCED.md)
4. **Troubleshoot Issues**: See [TROUBLESHOOTING_ENHANCED.md](TROUBLESHOOTING_ENHANCED.md)

---

## 📞 Support

**If installation fails:**

1. Check this guide again (step-by-step)
2. Try alternative installations (see troubleshooting)
3. Check GitHub issues: https://github.com/ML-bot-2012/ALFRID-the-ROS2-BUTLER/issues
4. Create new issue with:
   - Error message (copy-paste exact error)
   - What step failed
   - Your hardware (Pi model, OS version)
   - What you tried to fix it

---

**Last Updated**: January 31, 2026  
**Version**: 2.0 (January 2026 Production Release)
