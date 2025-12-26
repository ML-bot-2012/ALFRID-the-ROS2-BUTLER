# Motor Control Setup & Code

## Hardware
- L298N Dual H-Bridge Motor Driver
- 2x 12V DC Encoder Gear Motors
- Pi3B+ GPIO pins for control
- 12V Battery power

## GPIO Pin Mapping (Pi3B+)

### Left Motor
- GPIO 27 = Forward (IN1)
- GPIO 17 = Backward (IN2)
- GPIO 12 = PWM Speed (ENA)

### Right Motor
- GPIO 25 = Forward (IN3)
- GPIO 5 = Backward (IN4)
- GPIO 18 = PWM Speed (ENB)

## Motor Control Node Code

### motor_control_node.py
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
import time

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control')
        
        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Motor pins for Pi 3B+
        self.left_forward = 27
        self.left_backward = 17
        self.left_pwm_pin = 12
        
        self.right_forward = 25
        self.right_backward = 5
        self.right_pwm_pin = 18
        
        # Setup pins as outputs
        GPIO.setup([self.left_forward, self.left_backward, self.left_pwm_pin, self.right_forward, self.right_backward, self.right_pwm_pin], GPIO.OUT)
        
        # Setup PWM (1000 Hz frequency)
        self.left_pwm = GPIO.PWM(self.left_pwm_pin, 1000)
        self.right_pwm = GPIO.PWM(self.right_pwm_pin, 1000)
        
        self.left_pwm.start(0)
        self.right_pwm.start(0)
        
        # Ensure motors are OFF at startup
        self.set_motor_speed('left', 0.0)
        self.set_motor_speed('right', 0.0)
        
        # Subscribe to cmd_vel from Pi5
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.get_logger().info('Motor Control Node started (Pi 3B+ GPIO)')
    
    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z
        
        left_speed = linear - angular
        right_speed = linear + angular
        
        self.set_motor_speed('left', left_speed)
        self.set_motor_speed('right', right_speed)
    
    def set_motor_speed(self, side, speed):
        if side == 'left':
            forward_pin = self.left_forward
            backward_pin = self.left_backward
            pwm = self.left_pwm
        else:
            forward_pin = self.right_forward
            backward_pin = self.right_backward
            pwm = self.right_pwm
        
        # Limit speed to [-1.0, 1.0] and scale to motor range
        speed = max(-1.0, min(1.0, speed)) * 0.05
        
        if speed > 0:
            GPIO.output(forward_pin, GPIO.HIGH)
            GPIO.output(backward_pin, GPIO.LOW)
            pwm.ChangeDutyCycle(speed * 100)
        elif speed < 0:
            GPIO.output(forward_pin, GPIO.LOW)
            GPIO.output(backward_pin, GPIO.HIGH)
            pwm.ChangeDutyCycle(abs(speed) * 100)
        else:
            GPIO.output(forward_pin, GPIO.LOW)
            GPIO.output(backward_pin, GPIO.LOW)
            pwm.ChangeDutyCycle(0)
    
    def destroy_node(self):
        self.left_pwm.stop()
        self.right_pwm.stop()
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Installation

### On Pi3B+
```bash
# Install dependencies
sudo apt install -y python3-pip
sudo pip3 install RPi.GPIO

# Copy motor_control_node.py to:
~/butler_ros2_ws/src/butler_gpio/butler_gpio/motor_control_node.py

# Build
cd ~/butler_ros2_ws
colcon build

# Source setup
source /opt/ros/humble/setup.bash
source ~/butler_ros2_ws/install/setup.bash
```

## Launch Motor Control

### Terminal on Pi3B+
```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
ros2 run butler_gpio motor_control_node
```

## Test Motor Control

### Method 1: Manual Command
```bash
# On Pi5, send twist command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

### Method 2: Teleop
```bash
# Run teleop script
python3 ~/butler_ros2_ws/src/butler_control/butler_control/teleop_node.py

# Keys:
# W = Forward
# S = Backward
# A = Left
# D = Right
# SPACE = Stop
# Q = Quit
```

### Method 3: Test Script
```bash
sudo python3 ~/test_motor.py
```

## Motor Speed Control

### Speed Values
- 0.0 = Stop
- 0.1 = Slow forward
- 0.15 = Medium forward
- 0.2+ = Fast forward (capped at 0.05 * 100% in code)
- -0.1 = Slow backward
- -0.15 = Medium backward

### Adjust Speed Scaling
In `motor_control_node.py`, change this line:
```python
speed = max(-1.0, min(1.0, speed)) * 0.05  # Change 0.05 to higher/lower
```

- Increase to 0.1 for faster motors
- Decrease to 0.02 for slower motors

## Troubleshooting

### Motors Not Spinning
```bash
# Check GPIO is working
sudo python3 ~/test_motor.py

# Verify wiring (L298N pins)
# Check 12V battery power

# Check motor control node is running
ros2 node list | grep motor_control

# Check /cmd_vel topic is publishing
ros2 topic echo /cmd_vel
```

### Motor Spinning Wrong Direction
```bash
# Swap forward/backward pins in code
# Change IN1 and IN2 connections on L298N

# Or modify code:
if speed > 0:
    GPIO.output(forward_pin, GPIO.LOW)   # Swap
    GPIO.output(backward_pin, GPIO.HIGH) # Swap
```

### Jerky Movement
```bash
# Increase PWM frequency in code (currently 1000 Hz)
self.left_pwm = GPIO.PWM(self.left_pwm_pin, 2000)

# Or reduce motor speed commands
# Decrease speed scaling from 0.05 to 0.03
```

### Motor Overheating
```bash
# Reduce continuous power
# Lower speed scaling value
# Add cooling breaks between commands
```

## Performance Specs

### Motor Specifications
- Type: 12V DC Encoder Gear Motor
- Voltage: 12V DC
- Current: ~1A per motor (max)
- Speed: Variable with PWM
- Torque: High (geared)
- Max RPM: ~300 (no load)

### Control Parameters
- PWM Frequency: 1000 Hz
- PWM Resolution: 0-100%
- Max Speed Scaling: 0.05 (adjustable)
- Response Time: < 50ms

## ROS2 Topics

### Subscribe
- `/cmd_vel` (geometry_msgs/msg/Twist) - Motor commands

## Safety Notes
- Always ensure motors are OFF at startup
- Never run PWM at 100% continuously
- Check battery voltage before running
- Stop motors if any unusual noise occurs
- Verify all connections before powering on

Property of 5KROBOTICS & MALHAR LABADE © 2025
