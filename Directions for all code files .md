# ALFRID Robot - Directions for All Code Files - Beginner's Tutorial

**Property of 5KROBOTICS & MALHAR LABADE © 2026**

## ⚠️ Important Message

This setup is **complex** - it involves two Raspberry Pis running different ROS2 distributions, motor control, encoders, SLAM, and navigation. **DO NOT WORRY!** 

Just follow the commands exactly as written. Every step is tested and will work. If you get stuck, read the troubleshooting section. The journey is long but straightforward - copy, paste, and execute. You've got this! 💪

---

## Table of Contents
1. [Initial Setup](#initial-setup)
2. [Create ROS2 Package](#create-ros2-package)
3. [Create All Files](#create-all-files)
4. [Build with Colcon](#build-with-colcon)
5. [Launch and Test](#launch-and-test)
6. [Troubleshooting](#troubleshooting)

---

## Initial Setup

### 🔴 ALFRIDCL (Pi3B+ - Motor Control)

**Run these commands on ALFRIDCL only**

```bash
# Open Terminal 1
ssh malharlabade@192.168.86.226

# Update system
sudo apt update && sudo apt upgrade -y

# Create workspace directory
mkdir -p ~/butler_ros2_ws/src
cd ~/butler_ros2_ws

# Verify you're in the right place
pwd
# Should output: /home/malharlabade/butler_ros2_ws
```

### 🔵 ALFRIDROS (Pi5 - Navigation)

**Run these commands on ALFRIDROS only**

```bash
# Open Terminal 2
ssh malharlabade@192.168.86.222

# Create workspace directory
mkdir -p ~/butler_ros2_ws/src
cd ~/butler_ros2_ws

# Verify directory
pwd
# Should output: /home/malharlabade/butler_ros2_ws
```

---

## Create ROS2 Package (ALFRIDCL ONLY)

### 🔴 ALFRIDCL (Pi3B+)

**Run these commands on ALFRIDCL only**

```bash
# Make sure you're on ALFRIDCL
# Terminal should show: malharlabade@ALFRIDCL:~/butler_ros2_ws$

# Navigate to src directory
cd ~/butler_ros2_ws/src

# Create package called butler_gpio
ros2 pkg create butler_gpio --build-type ament_python

# Verify package created
ls -la butler_gpio/
# Should show: 
# butler_gpio/
# ├── butler_gpio/
# ├── test/
# ├── package.xml
# ├── setup.py
# ├── setup.cfg
# └── resource/
```

---

## Create All Files

### File 1: Create `package.xml` on ALFRIDCL

### 🔴 ALFRIDCL (Pi3B+)

**Location:** `~/butler_ros2_ws/src/butler_gpio/package.xml`

```bash
# Open file editor
nano ~/butler_ros2_ws/src/butler_gpio/package.xml

# Delete all content and replace with:
```

```xml
<?xml version="1.0"?>
<package format="3">
  <name>butler_gpio</name>
  <version>0.1.0</version>
  <description>Butler robot GPIO control for Pi 3B+ (Humble)</description>
  <maintainer email="butler@robot.local">butler</maintainer>
  <license>Apache License 2.0</license>
  
  <buildtool_depend>ament_python</buildtool_depend>
  <build_depend>rclpy</build_depend>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>tf2_ros</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

**How to save in nano:**
- Press `Ctrl+X`
- Press `Y` for yes
- Press `Enter` to confirm

---

### File 2: Create `setup.py` on ALFRIDCL

### 🔴 ALFRIDCL (Pi3B+)

**Location:** `~/butler_ros2_ws/src/butler_gpio/setup.py`

```bash
nano ~/butler_ros2_ws/src/butler_gpio/setup.py
```

```python
from setuptools import setup

package_name = 'butler_gpio'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/pi3b_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='butler',
    maintainer_email='butler@robot.local',
    description='Butler robot GPIO control for Pi 3B+ (Humble)',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'motor_control_node = butler_gpio.motor_control_node:main',
            'encoder_odometry_node = butler_gpio.encoder_odometry_node:main',
        ],
    },
)
```

---

### File 3: Create `motor_control_node.py` on ALFRIDCL

### 🔴 ALFRIDCL (Pi3B+)

**Location:** `~/butler_ros2_ws/src/butler_gpio/butler_gpio/motor_control_node.py`

```bash
nano ~/butler_ros2_ws/src/butler_gpio/butler_gpio/motor_control_node.py
```

```python
#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

class MotorControlNode:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('motor_control')
        self.subscription = self.node.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # LEFT MOTOR
        self.LEFT_FWD = 25      
        self.LEFT_BCK = 5     
        self.LEFT_PWM_PIN = 6
        
        # RIGHT MOTOR
        self.RIGHT_FWD = 23
        self.RIGHT_BCK = 24
        self.RIGHT_PWM_PIN = 22
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        for pin in [self.LEFT_FWD, self.LEFT_BCK, self.LEFT_PWM_PIN, self.RIGHT_FWD, self.RIGHT_BCK, self.RIGHT_PWM_PIN]:
            GPIO.setup(pin, GPIO.OUT)
        
        self.left_pwm = GPIO.PWM(self.LEFT_PWM_PIN, 1000)
        self.right_pwm = GPIO.PWM(self.RIGHT_PWM_PIN, 1000)
        self.left_pwm.start(0)
        self.right_pwm.start(0)
    
    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # W pressed - FORWARD (both motors forward)
        if linear_x > 0 and angular_z == 0:
            print("W pressed - FORWARD")
            GPIO.output(self.LEFT_FWD, GPIO.HIGH)
            GPIO.output(self.LEFT_BCK, GPIO.LOW)
            self.left_pwm.ChangeDutyCycle(90)
            
            GPIO.output(self.RIGHT_FWD, GPIO.HIGH)
            GPIO.output(self.RIGHT_BCK, GPIO.LOW)
            self.right_pwm.ChangeDutyCycle(80)
        
        # S pressed - TURN AROUND (both motors backward, left faster)
        elif linear_x < 0 and angular_z == 0:
            print("S pressed - TURN AROUND")
            GPIO.output(self.LEFT_FWD, GPIO.LOW)
            GPIO.output(self.LEFT_BCK, GPIO.HIGH)
            self.left_pwm.ChangeDutyCycle(100)  # Faster
            
            GPIO.output(self.RIGHT_FWD, GPIO.LOW)
            GPIO.output(self.RIGHT_BCK, GPIO.HIGH)
            self.right_pwm.ChangeDutyCycle(70)  # Slower
        
        # A pressed - TURN LEFT (left slower, right faster - both forward)
        elif angular_z > 0 and linear_x == 0:
            print("A pressed - TURN LEFT")
            GPIO.output(self.LEFT_FWD, GPIO.HIGH)
            GPIO.output(self.LEFT_BCK, GPIO.LOW)
            self.left_pwm.ChangeDutyCycle(70)  # Slower
            
            GPIO.output(self.RIGHT_FWD, GPIO.HIGH)
            GPIO.output(self.RIGHT_BCK, GPIO.LOW)
            self.right_pwm.ChangeDutyCycle(100)  # Faster
        
        # D pressed - TURN RIGHT (right slower, left faster - both forward)
        elif angular_z < 0 and linear_x == 0:
            print("D pressed - TURN RIGHT")
            GPIO.output(self.LEFT_FWD, GPIO.HIGH)
            GPIO.output(self.LEFT_BCK, GPIO.LOW)
            self.left_pwm.ChangeDutyCycle(100)  # Faster
            
            GPIO.output(self.RIGHT_FWD, GPIO.HIGH)
            GPIO.output(self.RIGHT_BCK, GPIO.LOW)
            self.right_pwm.ChangeDutyCycle(70)  # Slower
        
        # STOP
        else:
            print("STOP")
            GPIO.output(self.LEFT_FWD, GPIO.LOW)
            GPIO.output(self.LEFT_BCK, GPIO.LOW)
            self.left_pwm.ChangeDutyCycle(0)
            
            GPIO.output(self.RIGHT_FWD, GPIO.LOW)
            GPIO.output(self.RIGHT_BCK, GPIO.LOW)
            self.right_pwm.ChangeDutyCycle(0)
    
    def run(self):
        try:
            rclpy.spin(self.node)
        except KeyboardInterrupt:
            self.stop_all_motors()
            GPIO.cleanup()
            self.node.destroy_node()
            rclpy.shutdown()
    
    def stop_all_motors(self):
        GPIO.output(self.LEFT_FWD, GPIO.LOW)
        GPIO.output(self.LEFT_BCK, GPIO.LOW)
        GPIO.output(self.RIGHT_FWD, GPIO.LOW)
        GPIO.output(self.RIGHT_BCK, GPIO.LOW)
        self.left_pwm.stop()
        self.right_pwm.stop()


def main():
    node = MotorControlNode()
    node.run()

if __name__ == '__main__':
    main()
```

---

### File 4: Create `encoder_odometry_node.py` on ALFRIDCL

### 🔴 ALFRIDCL (Pi3B+)

**Location:** `~/butler_ros2_ws/src/butler_gpio/butler_gpio/encoder_odometry_node.py`

```bash
nano ~/butler_ros2_ws/src/butler_gpio/butler_gpio/encoder_odometry_node.py
```

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import RPi.GPIO as GPIO
import math

class EncoderOdometryNode(Node):
    def __init__(self):
        super().__init__('encoder_odometry')
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        self.wheel_diameter = 0.08
        self.encoder_resolution = 660
        self.wheel_base = 0.20
        self.meters_per_count = (math.pi * self.wheel_diameter) / self.encoder_resolution
        self.left_speed_scale = 1.0
        self.right_speed_scale = 1.0
        
        # Encoder pins (RF, LR, LF, RR)
        self.rf_a_pin = 12
        self.rf_b_pin = 11
        self.lr_a_pin = 4
        self.lr_b_pin = 8
        self.lf_a_pin = 10
        self.lf_b_pin = 7
        self.rr_a_pin = 15
        self.rr_b_pin = 14
        
        all_pins = [self.rf_a_pin, self.rf_b_pin, self.lr_a_pin, self.lr_b_pin, 
                    self.lf_a_pin, self.lf_b_pin, self.rr_a_pin, self.rr_b_pin]
        GPIO.setup(all_pins, GPIO.IN)
        
        self.rf_count = 0
        self.lr_count = 0
        self.lf_count = 0
        self.rr_count = 0
        
        self.last_rf_count = 0
        self.last_lr_count = 0
        self.last_lf_count = 0
        self.last_rr_count = 0
        
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        
        GPIO.add_event_detect(self.rf_a_pin, GPIO.RISING, callback=self.rf_callback, bouncetime=2)
        GPIO.add_event_detect(self.lr_a_pin, GPIO.RISING, callback=self.lr_callback, bouncetime=2)
        GPIO.add_event_detect(self.lf_a_pin, GPIO.RISING, callback=self.lf_callback, bouncetime=2)
        GPIO.add_event_detect(self.rr_a_pin, GPIO.RISING, callback=self.rr_callback, bouncetime=2)
        
        self.odom_pub = self.create_publisher(Odometry, 'encoder_odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(0.05, self.odometry_callback)
        self.get_logger().info('Encoder Odometry Node started (4-encoder quadrature)')
    
    def rf_callback(self, channel):
        b_state = GPIO.input(self.rf_b_pin)
        self.rf_count += 1 if b_state == 0 else -1
    
    def lr_callback(self, channel):
        b_state = GPIO.input(self.lr_b_pin)
        self.lr_count += 1 if b_state == 0 else -1
    
    def lf_callback(self, channel):
        b_state = GPIO.input(self.lf_b_pin)
        self.lf_count += 1 if b_state == 0 else -1
    
    def rr_callback(self, channel):
        b_state = GPIO.input(self.rr_b_pin)
        self.rr_count += 1 if b_state == 0 else -1
    
    def odometry_callback(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        if dt == 0:
            return
        
        delta_lf = self.lf_count - self.last_lf_count
        delta_lr = self.lr_count - self.last_lr_count
        delta_rf = self.rf_count - self.last_rf_count
        delta_rr = self.rr_count - self.last_rr_count
        
        self.last_lf_count = self.lf_count
        self.last_lr_count = self.lr_count
        self.last_rf_count = self.rf_count
        self.last_rr_count = self.rr_count
        
        left_distance = (delta_lf + delta_lr) / 2 * self.meters_per_count * self.left_speed_scale
        right_distance = (delta_rf + delta_rr) / 2 * self.meters_per_count * self.right_speed_scale
        
        linear_x = (left_distance + right_distance) / 2
        angular_z = (right_distance - left_distance) / self.wheel_base
        
        self.x += linear_x * math.cos(self.theta)
        self.y += linear_x * math.sin(self.theta)
        self.theta += angular_z
        
        while self.theta > math.pi:
            self.theta -= 2 * math.pi
        while self.theta < -math.pi:
            self.theta += 2 * math.pi
        
        q = self.euler_to_quaternion(0, 0, self.theta)
        
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        
        self.odom_pub.publish(odom)
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy
        
        return [qx, qy, qz, qw]


def main():
    node = EncoderOdometryNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

---

### File 5: Create `pi3b_launch.py` on ALFRIDCL

### 🔴 ALFRIDCL (Pi3B+)

**Location:** `~/butler_ros2_ws/src/butler_gpio/launch/pi3b_launch.py`

First create the launch directory:
```bash
mkdir -p ~/butler_ros2_ws/src/butler_gpio/launch
```

Then create the file:
```bash
nano ~/butler_ros2_ws/src/butler_gpio/launch/pi3b_launch.py
```

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='butler_gpio',
            executable='motor_control_node',
            name='motor_control',
            output='screen',
        ),
        Node(
            package='butler_gpio',
            executable='encoder_odometry_node',
            name='encoder_odometry',
            output='screen',
        ),
    ])
```

---

### File 6: Create `butler.urdf` on ALFRIDROS

### 🔵 ALFRIDROS (Pi5)

**Location:** `~/butler_ros2_ws/src/butler_control/urdf/butler.urdf`

First create the directories:
```bash
# On ALFRIDROS
mkdir -p ~/butler_ros2_ws/src/butler_control/urdf
```

Then create the file:
```bash
nano ~/butler_ros2_ws/src/butler_control/urdf/butler.urdf
```

```xml
<?xml version="1.0"?>
<robot name="butler">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.25 0.25 0.35"/>
      </geometry>
      <origin xyz="0 0 0.175"/>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.25 0.25 0.35"/>
      </geometry>
      <origin xyz="0 0 0.175"/>
    </collision>
  </link>

  <link name="top_plate">
    <visual>
      <geometry>
        <box size="0.27 0.27 0.02"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
  </link>

  <joint name="top_plate_joint" type="fixed">
    <parent link="base_link"/>
    <child link="top_plate"/>
    <origin xyz="0 0 0.37"/>
  </joint>

  <link name="laser">
    <visual>
      <geometry>
        <cylinder radius="0.06" length="0.08"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
  </link>

  <joint name="laser_joint" type="fixed">
    <parent link="top_plate"/>
    <child link="laser"/>
    <origin xyz="0 0 0.05"/>
  </joint>

  <link name="left_front_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.02"/>
      </geometry>
      <origin rpy="0 1.5708 1.5708"/>
      <material name="dark_gray">
        <color rgba="0.2 0.2 0.2 1"/>
      </material>
    </visual>
  </link>

  <joint name="left_front_wheel_joint" type="fixed">
    <parent link="base_link"/>
    <child link="left_front_wheel"/>
    <origin xyz="-0.05 0.125 0.0325"/>
  </joint>

  <link name="right_front_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.02"/>
      </geometry>
      <origin rpy="0 1.5708 1.5708"/>
      <material name="dark_gray">
        <color rgba="0.2 0.2 0.2 1"/>
      </material>
    </visual>
  </link>

  <joint name="right_front_wheel_joint" type="fixed">
    <parent link="base_link"/>
    <child link="right_front_wheel"/>
    <origin xyz="-0.05 -0.125 0.0325"/>
  </joint>

  <link name="left_rear_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.02"/>
      </geometry>
      <origin rpy="0 1.5708 1.5708"/>
      <material name="dark_gray">
        <color rgba="0.2 0.2 0.2 1"/>
      </material>
    </visual>
  </link>

  <joint name="left_rear_wheel_joint" type="fixed">
    <parent link="base_link"/>
    <child link="left_rear_wheel"/>
    <origin xyz="0.05 0.125 0.0325"/>
  </joint>

  <link name="right_rear_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.02"/>
      </geometry>
      <origin rpy="0 1.5708 1.5708"/>
      <material name="dark_gray">
        <color rgba="0.2 0.2 0.2 1"/>
      </material>
    </visual>
  </link>

  <joint name="right_rear_wheel_joint" type="fixed">
    <parent link="base_link"/>
    <child link="right_rear_wheel"/>
    <origin xyz="0.05 -0.125 0.0325"/>
  </joint>
</robot>
```

---

### File 7: Create `teleop_simple.py` on ALFRIDCL

### 🔴 ALFRIDCL (Pi3B+)

**Location:** `~/teleop_simple.py`

```bash
nano ~/teleop_simple.py
```

```python
#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Twist
import sys
import tty
import termios

rclpy.init()
node = rclpy.create_node('teleop')
pub = node.create_publisher(Twist, 'cmd_vel', 10)

print("\n" + "="*60)
print("ALFRID Teleop Control")
print("="*60)
print("W - Forward")
print("S - Turn Around")
print("A - Turn Left")
print("D - Turn Right")
print("SPACE - Stop")
print("Q - Quit\n")

def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

while True:
    key = get_key().lower()
    twist = Twist()
    
    if key == 'w':
        twist.linear.x = 0.5
        twist.angular.z = 0.0
        print("FORWARD")
    elif key == 's':
        twist.linear.x = -0.5
        twist.angular.z = 0.0
        print("TURN AROUND")
    elif key == 'a':
        twist.linear.x = 0.0
        twist.angular.z = 0.5
        print("TURN LEFT")
    elif key == 'd':
        twist.linear.x = 0.0
        twist.angular.z = -0.5
        print("TURN RIGHT")
    elif key == ' ':
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        print("STOP")
    elif key == 'q':
        print("Quit")
        break
    else:
        continue
    
    pub.publish(twist)

rclpy.shutdown()
```

Make it executable:
```bash
chmod +x ~/teleop_simple.py
```

---

## Build with Colcon

### Step 1: Verify File Structure on ALFRIDCL

### 🔴 ALFRIDCL (Pi3B+)

**Run these commands on ALFRIDCL only**

```bash
# Check all files are in place
ls -la ~/butler_ros2_ws/src/butler_gpio/
# Should show:
# butler_gpio/
# ├── butler_gpio/
# │   ├── __init__.py
# │   ├── motor_control_node.py
# │   └── encoder_odometry_node.py
# ├── launch/
# │   └── pi3b_launch.py
# ├── test/
# ├── package.xml
# ├── setup.py
# └── setup.cfg
```

### Step 2: Source ROS2 on ALFRIDCL

### 🔴 ALFRIDCL (Pi3B+)

**Run these commands on ALFRIDCL only**

```bash
# Add to ~/.bashrc for every new terminal (optional but recommended)
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Or manually source in current terminal
source /opt/ros/humble/setup.bash
```

### Step 3: Build with Colcon on ALFRIDCL

### 🔴 ALFRIDCL (Pi3B+)

**Run these commands on ALFRIDCL only**

```bash
# Navigate to workspace
cd ~/butler_ros2_ws

# Build only butler_gpio package
colcon build --packages-select butler_gpio

# Wait for build to complete
# You should see:
# Summary: 1 package finished [XX.Xs]
```

### Step 4: Source Install on ALFRIDCL

### 🔴 ALFRIDCL (Pi3B+)

**Run these commands on ALFRIDCL only**

```bash
# This makes the package available to run
source ~/butler_ros2_ws/install/setup.bash

# Add to ~/.bashrc for convenience
echo "source ~/butler_ros2_ws/install/setup.bash" >> ~/.bashrc
```

### Step 5: Verify Build on ALFRIDCL

### 🔴 ALFRIDCL (Pi3B+)

**Run these commands on ALFRIDCL only**

```bash
# Check if executables were created
ls -la ~/butler_ros2_ws/install/butler_gpio/lib/butler_gpio/

# Should show:
# motor_control_node (executable)
# encoder_odometry_node (executable)
```

---

## Launch and Test

### Terminal 1 (ALFRIDCL): Motor Control

### 🔴 ALFRIDCL (Pi3B+)

**Run this on ALFRIDCL in Terminal 1**

```bash
cd ~/butler_ros2_ws
sudo bash -c "source /opt/ros/humble/setup.bash && unset RMW_IMPLEMENTATION && export ROS_DOMAIN_ID=0 && source install/setup.bash && ros2 launch butler_gpio pi3b_launch.py"

# Expected output:
# [motor_control_node-1]: process started with pid [XXXX]
# [encoder_odometry_node-2]: process started with pid [XXXX]
# [INFO] [encoder_odometry]: Encoder Odometry Node started
```

**Keep this terminal running!**

---

### Terminal 2 (ALFRIDCL): Teleop Control

### 🔴 ALFRIDCL (Pi3B+)

**Run this on ALFRIDCL in Terminal 2**

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
python3 ~/teleop_simple.py

# Expected output:
# ============================================================
# ALFRID Teleop Control
# ============================================================
# W - Forward
# S - Turn Around
# A - Turn Left
# D - Turn Right
# SPACE - Stop
# Q - Quit
```

**Now press W, A, S, D to test motor control!**

---

### Terminal 3 (ALFRIDROS): Transforms + RPLidar + RSP

### 🔵 ALFRIDROS (Pi5)

**Run this on ALFRIDROS in Terminal 3**

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Full command (copy and paste entire block):
(nohup ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom > /tmp/tf1.log 2>&1 & nohup ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link > /tmp/tf2.log 2>&1 & nohup ros2 run tf2_ros static_transform_publisher 0.08 0 0.23 0 0 0 base_link laser > /tmp/tf3.log 2>&1 & sleep 1 && nohup ros2 run rplidar_ros rplidar_composition --ros-args -p serial_port:=/dev/ttyUSB0 -p frame_id:=laser > /tmp/rplidar.log 2>&1 & sleep 2 && ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat ~/butler_ros2_ws/src/butler_control/urdf/butler.urdf)")
```

---

### Terminal 4 (ALFRIDROS): SLAM Toolbox

### 🔵 ALFRIDROS (Pi5)

**Run this on ALFRIDROS in Terminal 4**

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run slam_toolbox async_slam_toolbox_node --ros-args -p use_sim_time:=false
```

---

### Terminal 5 (ALFRIDROS): Nav2 + RViz

### 🔵 ALFRIDROS (Pi5)

**Run this on ALFRIDROS in Terminal 5**

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false
```

### Build Fails with "No such file or directory"

```bash
# Make sure you're in the workspace root
cd ~/butler_ros2_ws

# Check files exist
find . -name "package.xml"
find . -name "setup.py"

# If missing, create them using nano as shown above
```

### Colcon Build Complains About Missing Dependencies

```bash
# Install missing dependencies
sudo apt install python3-rclpy python3-geometry-msgs python3-nav-msgs python3-tf2-ros

# Then try build again
cd ~/butler_ros2_ws
colcon build --packages-select butler_gpio
```

### Motor Control Node Won't Start

```bash
# Check if sudo is needed (it is!)
sudo bash -c 'source /opt/ros/humble/setup.bash && source ~/butler_ros2_ws/install/setup.bash && ros2 launch butler_gpio pi3b_launch.py'

# OR run directly:
source /opt/ros/humble/setup.bash
source ~/butler_ros2_ws/install/setup.bash
sudo ros2 launch butler_gpio pi3b_launch.py
```

### "command not found: colcon"

```bash
# Install colcon
sudo apt install python3-colcon-common-extensions

# Verify
colcon --version
```

### RPLidar Not Spinning

```bash
# Check USB connection
ls -la /dev/ttyUSB*

# Should show /dev/ttyUSB0
# If different, update the serial_port parameter in the command
```

---

## Summary Checklist

- [ ] Created `~/butler_ros2_ws/src/butler_gpio/package.xml`
- [ ] Created `~/butler_ros2_ws/src/butler_gpio/setup.py`
- [ ] Created `~/butler_ros2_ws/src/butler_gpio/butler_gpio/motor_control_node.py`
- [ ] Created `~/butler_ros2_ws/src/butler_gpio/butler_gpio/encoder_odometry_node.py`
- [ ] Created `~/butler_ros2_ws/src/butler_gpio/launch/pi3b_launch.py`
- [ ] Created `~/butler_ros2_ws/src/butler_control/urdf/butler.urdf`
- [ ] Created `~/teleop_simple.py`
- [ ] Ran `colcon build --packages-select butler_gpio`
- [ ] Sourced `/opt/ros/humble/setup.bash`
- [ ] Sourced `~/butler_ros2_ws/install/setup.bash`
- [ ] Motor control node launches
- [ ] Encoder odometry publishes
- [ ] Teleop controls robot

---

**Property of 5KROBOTICS & MALHAR LABADE © 2026**
