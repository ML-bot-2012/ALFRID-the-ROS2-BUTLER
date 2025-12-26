# Motor Control Setup & Code

## Hardware
- L298N Dual H-Bridge Motor Driver
- 2x 12V DC Encoder Gear Motors
- Pi3B+ GPIO pins for control
- 12V Battery power

## GPIO Pin Mapping (ALFRIDCL - Pi3B+) - CORRECTED

### Left Motor
- GPIO 5 = Forward (IN1)
- GPIO 25 = Backward (IN2)
- GPIO 6 = PWM Speed (ENA)

### Right Motor
- GPIO 23 = Forward (IN3)
- GPIO 24 = Backward (IN4)
- GPIO 22 = PWM Speed (ENB)

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
        
        # Motor pins for Pi 3B+ (CORRECTED)
        self.left_forward = 5      # IN1
        self.left_backward = 25    # IN2
        self.left_pwm_pin = 6      # ENA
        
        self.right_forward = 23    # IN3
        self.right_backward = 24   # IN4
        self.right_pwm_pin = 22    # ENB
        
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
        
        # Subscribe to cmd_vel from ALFRIDROS (Pi5)
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.get_logger().info('Motor Control Node started (ALFRIDCL Pi 3B+ GPIO)')
    
    def cmd_vel_callback(self, msg):
        """Handle velocity commands from ROS2"""
        linear = msg.linear.x
        angular = msg.angular.z
        
        left_speed = linear - angular
        right_speed = linear + angular
        
        self.set_motor_speed('left', left_speed)
        self.set_motor_speed('right', right_speed)
    
    def set_motor_speed(self, side, speed):
        """Set motor speed and direction"""
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
            # Forward
            GPIO.output(forward_pin, GPIO.HIGH)
            GPIO.output(backward_pin, GPIO.LOW)
            pwm.ChangeDutyCycle(speed * 100)
        elif speed < 0:
            # Backward
            GPIO.output(forward_pin, GPIO.LOW)
            GPIO.output(backward_pin, GPIO.HIGH)
            pwm.ChangeDutyCycle(abs(speed) * 100)
        else:
            # Stop
            GPIO.output(forward_pin, GPIO.LOW)
            GPIO.output(backward_pin, GPIO.LOW)
            pwm.ChangeDutyCycle(0)
    
    def destroy_node(self):
        """Cleanup GPIO on shutdown"""
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

### On ALFRIDCL (Pi3B+)
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

### Terminal on ALFRIDCL
```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
ros2 run butler_gpio motor_control_node
```

## Test Motor Control

### Method 1: Manual Command
```bash
# On ALFRIDROS (Pi5), send twist command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

### Method 2: Teleop Control
```bash
# Run teleop script on ALFRIDROS
python3 ~/butler_ros2_ws/src/butler_control/butler_control/teleop_node.py

# Keys:
# W = Forward
# S = Backward
# A = Left Turn
# D = Right Turn
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

## L298N Wiring Reference

### Motor Connections
LEFT MOTOR:
GPIO 5 (IN1) → L298N IN1 (Left Forward)
GPIO 25 (IN2) → L298N IN2 (Left Backward)
GPIO 6 (ENA) → L298N ENA (Left PWM)
L298N OUT1 & OUT2 → Left Motor
RIGHT MOTOR:
GPIO 23 (IN3) → L298N IN3 (Right Forward)
GPIO 24 (IN4) → L298N IN4 (Right Backward)
GPIO 22 (ENB) → L298N ENB (Right PWM)
L298N OUT3 & OUT4 → Right Motor
POWER:
L298N GND → ALFRIDCL GND
12V Battery + → L298N +12V
12V Battery - → GND (shared ground)

## ROS2 Topics

### Subscribe
- `/cmd_vel` (geometry_msgs/msg/Twist) - Motor velocity commands

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

### GPIO Permission Denied
```bash
sudo pkill -9 python3
sudo reboot
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
- Update Rate: 10 Hz (from ROS2 topic)

## Safety Notes
- Always ensure motors are OFF at startup
- Never run PWM at 100% continuously
- Check battery voltage before running (should be ~12V)
- Stop motors if any unusual noise occurs
- Verify all connections before powering on
- Disconnect battery before making wiring changes

## GPIO Pin Summary
GPIO 5  = Left Motor Forward (IN1)
GPIO 25 = Left Motor Backward (IN2)
GPIO 6  = Left Motor PWM (ENA)
GPIO 23 = Right Motor Forward (IN3)
GPIO 24 = Right Motor Backward (IN4)
GPIO 22 = Right Motor PWM (ENB)

## Integration with ALFRID System

### Motor Control in SLAM
Motors respond to `/cmd_vel` published by:
- Teleop node (manual control)
- Nav2 navigation stack (autonomous)
- SLAM exploration

### Motor Control with Limit Switches
When limit switches trigger (GPIO 17/27), motor commands can be overridden by servo food dispenser

## Future Enhancements
- [ ] Add motor speed feedback
- [ ] Implement soft start to reduce jerking
- [ ] Add motor current monitoring
- [ ] Implement motor fault detection
- [ ] Add dynamic speed scaling based on battery voltage

Property of 5KROBOTICS & MALHAR LABADE © 2025
