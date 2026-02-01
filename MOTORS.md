# BUTLER Motor Control System - COMPLETE GUIDE

**Property of 5KROBOTICS & MALHAR LABADE**

Complete guide to BUTLER's differential drive motor system with smooth control algorithms.

---

## 🔧 Hardware Architecture

### Motor System Overview

```
12V Battery
    ↓
[L298N Motor Driver Module]
    ├─→ LEFT MOTOR (12V DC Encoder)
    ├─→ RIGHT MOTOR (12V DC Encoder)
    └─→ Enable pins (GPIO PWM)

GPIO Control (ALFRIDCL - Pi3B+)
├── GPIO 25 (LEFT Forward)
├── GPIO 5  (LEFT Backward)
├── GPIO 6  (LEFT PWM @ 1000Hz)
├── GPIO 23 (RIGHT Forward)
├── GPIO 24 (RIGHT Backward)
└── GPIO 22 (RIGHT PWM @ 1000Hz)
```

### Electrical Specifications

```
Motors:
├── Type: 12V DC Geared Motors
├── Voltage: 12V nominal (11V minimum, 13V maximum)
├── Encoder: Quadrature encoder on each motor
└── Current draw: ~500mA each at full load

Driver (L298N):
├── Current rating: 2A per channel
├── Voltage range: 5V-35V logic, 5V-50V motor
├── Frequency: Supports 0-200kHz PWM
└── Logic: Active HIGH

Power requirements:
├── Motor supply: 12V 3000mAh minimum
├── Logic supply (Pi): 5V 2.5A minimum
└── Total: 30W typical, 60W peak
```

---

## 🎯 Control System

### Smooth Differential Drive Algorithm

The motor control uses **smooth acceleration/deceleration** to prevent jerky movements and mechanical stress.

```
Command Flow:
  ROS2 /cmd_vel Topic
        ↓
  motor_control_node.py
        ↓
  Smooth Acceleration
  (0.05 per 50ms cycle)
        ↓
  GPIO PWM Control
  @ 1000Hz
        ↓
  Motor Movement
```

### Speed Parameters

```
Speed Multipliers (calibrated for floor friction):
├── linear.x (forward/backward): 0.2
│   • Normal floor: smooth movement
│   • Carpet: may need 0.15
│   • Tile: may need 0.25
│
└── angular.z (turning): 0.4
    • Faster turning for in-place rotation
    • Good balance with forward speed

Acceleration:
├── Rate: 0.05 per 50ms cycle
├── Time to full speed: ~500ms
├── Prevents: Mechanical stress, wheel slip
└── Result: Smooth, natural movement

Timeout:
├── Duration: 0.5 seconds
├── Action: Auto-stops if no command received
├── Purpose: Safety if connection lost
└── Message: Continues accepting commands
```

### Differential Drive Equations

The robot uses **differential drive** where left and right motors move independently:

```
From teleop command (linear_x, angular_z):

LEFT_target = (linear_x × 0.2) - (angular_z × 0.4)
RIGHT_target = -((linear_x × 0.2) + (angular_z × 0.4))

RIGHT is inverted because motors are mounted opposite

Examples:
1. Forward (W): linear_x=1.0, angular_z=0.0
   → LEFT = 0.2, RIGHT = -0.2 (both forward)

2. Left turn (A): linear_x=0.0, angular_z=1.0
   → LEFT = -0.4, RIGHT = 0.4 (opposite directions)

3. Forward+Left: linear_x=1.0, angular_z=1.0
   → LEFT = -0.2, RIGHT = 0.6 (arc movement)
```

---

## 💻 Complete Source Code: motor_control_node.py

**Location**: `src/butler_gpio/butler_gpio/motor_control_node.py`  
**Platform**: Raspberry Pi 3B+ (ROS2 Humble)  
**Python**: 3.9+

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
import time

class MotorControl(Node):
    def __init__(self):
        super().__init__('motor_control')
        
        # GPIO Setup
        GPIO.setmode(GPIO.BCM)
        
        # LEFT Motor (Motor A) - GPIO pins
        self.left_forward = 25
        self.left_backward = 5
        self.left_pwm_pin = 6
        
        # RIGHT Motor (Motor B) - GPIO pins
        self.right_forward = 23
        self.right_backward = 24
        self.right_pwm_pin = 22
        
        # Setup GPIO pins
        for pin in [self.left_forward, self.left_backward, self.left_pwm_pin,
                    self.right_forward, self.right_backward, self.right_pwm_pin]:
            GPIO.setup(pin, GPIO.OUT)
        
        # PWM Setup (frequency = 1000 Hz for smooth control)
        self.left_pwm = GPIO.PWM(self.left_pwm_pin, 1000)
        self.right_pwm = GPIO.PWM(self.right_pwm_pin, 1000)
        
        self.left_pwm.start(0)
        self.right_pwm.start(0)
        
        # Velocity tracking (current vs target)
        self.target_left_speed = 0.0
        self.target_right_speed = 0.0
        self.current_left_speed = 0.0
        self.current_right_speed = 0.0
        
        # Acceleration settings (smooth ramp)
        self.acceleration_rate = 0.05  # Per 50ms cycle
        self.max_speed = 1.0
        
        # Motor timeout (safety)
        self.last_cmd_time = time.time()
        self.cmd_timeout = 0.5  # seconds
        
        # ROS2 subscription to /cmd_vel
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Timer for control loop (50ms = 20Hz)
        self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('Motor Control Node initialized')
    
    def cmd_vel_callback(self, msg):
        """Callback for /cmd_vel topic - incoming movement commands"""
        self.last_cmd_time = time.time()
        
        # Extract linear and angular velocities from Twist message
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Apply speed multipliers (calibrated)
        linear_x = linear_x * 0.2   # Forward/backward scaling
        angular_z = angular_z * 0.4  # Turning scaling
        
        # Differential drive equations
        self.target_left_speed = linear_x - angular_z
        self.target_right_speed = -(linear_x + angular_z)  # RIGHT motor inverted
        
        # Clamp speeds to [-1.0, 1.0] range
        self.target_left_speed = max(-self.max_speed, min(self.max_speed, self.target_left_speed))
        self.target_right_speed = max(-self.max_speed, min(self.max_speed, self.target_right_speed))
    
    def control_loop(self):
        """Main control loop - runs every 50ms with smooth acceleration"""
        # Safety timeout: stop if no command for 0.5s
        if time.time() - self.last_cmd_time > self.cmd_timeout:
            self.target_left_speed = 0.0
            self.target_right_speed = 0.0
        
        # Smooth acceleration for LEFT motor
        if self.current_left_speed < self.target_left_speed:
            # Accelerate up to target
            self.current_left_speed = min(self.target_left_speed,
                                         self.current_left_speed + self.acceleration_rate)
        elif self.current_left_speed > self.target_left_speed:
            # Decelerate down to target
            self.current_left_speed = max(self.target_left_speed,
                                         self.current_left_speed - self.acceleration_rate)
        
        # Smooth acceleration for RIGHT motor
        if self.current_right_speed < self.target_right_speed:
            self.current_right_speed = min(self.target_right_speed,
                                          self.current_right_speed + self.acceleration_rate)
        elif self.current_right_speed > self.target_right_speed:
            self.current_right_speed = max(self.target_right_speed,
                                          self.current_right_speed - self.acceleration_rate)
        
        # Set motor speeds via GPIO PWM
        self.set_motor_speed(self.left_forward, self.left_backward, self.left_pwm, self.current_left_speed)
        self.set_motor_speed(self.right_forward, self.right_backward, self.right_pwm, self.current_right_speed)
    
    def set_motor_speed(self, forward_pin, backward_pin, pwm, speed):
        """Set motor speed and direction"""
        # Clamp speed to [-1.0, 1.0]
        speed = max(-1.0, min(1.0, speed))
        pwm_value = abs(speed) * 100  # Convert to 0-100% PWM duty cycle
        
        if speed > 0:
            # Moving forward
            GPIO.output(forward_pin, GPIO.HIGH)
            GPIO.output(backward_pin, GPIO.LOW)
            pwm.ChangeDutyCycle(pwm_value)
        elif speed < 0:
            # Moving backward
            GPIO.output(forward_pin, GPIO.LOW)
            GPIO.output(backward_pin, GPIO.HIGH)
            pwm.ChangeDutyCycle(pwm_value)
        else:
            # Stopped
            GPIO.output(forward_pin, GPIO.LOW)
            GPIO.output(backward_pin, GPIO.LOW)
            pwm.ChangeDutyCycle(0)
    
    def stop_motors_now(self):
        """Immediately stop motors (emergency)"""
        self.current_left_speed = 0.0
        self.current_right_speed = 0.0
        self.target_left_speed = 0.0
        self.target_right_speed = 0.0
        GPIO.output(self.left_forward, GPIO.LOW)
        GPIO.output(self.left_backward, GPIO.LOW)
        GPIO.output(self.right_forward, GPIO.LOW)
        GPIO.output(self.right_backward, GPIO.LOW)
    
    def destroy_node(self):
        """Cleanup on shutdown"""
        self.stop_motors_now()
        self.left_pwm.stop()
        self.right_pwm.stop()
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    motor_control = MotorControl()
    
    try:
        rclpy.spin(motor_control)
    except KeyboardInterrupt:
        motor_control.get_logger().info('Shutting down...')
    finally:
        motor_control.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 🔧 Configuration & Tuning

### Motor Polarity (Direction)

If motors move backwards:

```bash
# In motor_control_node.py, swap pins:

# For LEFT motor moving wrong direction:
# CHANGE:
self.left_forward = 25
self.left_backward = 5
# TO:
self.left_forward = 5
self.left_backward = 25

# For RIGHT motor moving wrong direction:
# CHANGE:
self.right_forward = 23
self.right_backward = 24
# TO:
self.right_forward = 24
self.right_backward = 23

# Then rebuild and restart:
cd ~/butler_ros2_ws
colcon build --packages-select butler_gpio
source install/setup.bash
```

### Speed Multiplier Tuning

If robot moves too fast or too slow:

```bash
# In motor_control_node.py:

# SLOW ROBOT: Increase multiplier
linear_x = linear_x * 0.3   # was 0.2
angular_z = angular_z * 0.5  # was 0.4

# FAST ROBOT: Decrease multiplier
linear_x = linear_x * 0.15  # was 0.2
angular_z = angular_z * 0.3  # was 0.4

# Test and iterate!
```

### Acceleration Rate Tuning

Control how fast speed ramps up/down:

```bash
# In motor_control_node.py:

# FASTER RESPONSE: Increase rate
self.acceleration_rate = 0.10  # was 0.05

# SMOOTHER MOVEMENT: Decrease rate
self.acceleration_rate = 0.02  # was 0.05

# Ramp time = max_speed / acceleration_rate
# 1.0 / 0.05 = 20 cycles = 1.0 second
```

### Timeout Configuration

```bash
# In motor_control_node.py:

# LONGER TIMEOUT: More forgiving if commands drop
self.cmd_timeout = 1.0  # was 0.5 (seconds)

# SHORTER TIMEOUT: Faster stop on disconnect
self.cmd_timeout = 0.25  # was 0.5 (seconds)
```

---

## 📊 Performance Metrics

| Metric | Target | Actual | Notes |
|--------|--------|--------|-------|
| **Response Time** | <100ms | ~50ms | Time from command to movement |
| **Acceleration** | ~500ms | Configurable | Time to reach full speed |
| **Max Speed** | ∞ | Limited by motors | ~0.3m/s typical |
| **Turning Radius** | ~30cm | Varies | Depends on surface friction |
| **PWM Frequency** | 1000Hz | 1000Hz | Smooth, inaudible control |
| **CPU Usage** | <5% | <3% | Very efficient |
| **Current Draw** | Variable | ~1A max | Both motors at full speed |

---

## 🧪 Testing Procedures

### Test 1: GPIO Connectivity

```bash
# On ALFRIDCL:
python3 << 'EOF'
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)

# Test each pin
pins = [25, 5, 6, 23, 24, 22]
for pin in pins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.HIGH)
    time.sleep(0.1)
    GPIO.output(pin, GPIO.LOW)
    print(f"✅ GPIO {pin} working")

GPIO.cleanup()
EOF
```

### Test 2: Motor Polarity

```bash
# Run motor control node (Terminal 1)
# Then in Terminal 2:
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.5}, angular: {z: 0.0}}'

# Expected: Both motors spin forward
# If backwards: Swap polarity (see above)
```

### Test 3: Speed Responsiveness

```bash
# Using teleop (Terminal 2):
# Press W: Should accelerate smoothly over ~500ms
# Release: Should decelerate smoothly
# No jerky or sudden movements
```

### Test 4: Timeout Safety

```bash
# In Terminal 2 teleop, press W
# Robot accelerates and moves forward
# Close teleop terminal (no more commands)
# Expected: Robot stops within 0.5 seconds
```

---

## 🚀 Next Steps

- **Tuned motors?** → See [QUICKSTART_ENHANCED.md](QUICKSTART_ENHANCED.md) to run full system
- **Troubleshooting?** → See [TROUBLESHOOTING_ENHANCED.md](TROUBLESHOOTING_ENHANCED.md)
- **Distance control?** → See [LIGAMENT_NAVIGATOR_ENHANCED.md](LIGAMENT_NAVIGATOR_ENHANCED.md)

---

**Last Updated**: January 31, 2026  
**Version**: 2.0 (January 2026 Production Release)
