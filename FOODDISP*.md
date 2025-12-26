# Limit Switch Yes/No + Servo Food Dispenser System

## Overview
Limit switches act as YES/NO buttons. When triggered, they control a servo motor with rack & pinion mechanism to dispense food on a tray.

## Hardware Components

### Limit Switches (Yes/No Input)
- 2x Mechanical limit switches (normally open)
- 2x 10kΩ pull-down resistors

### Servo Motor (Food Dispenser)
- 1x Servo motor (180° or 270°)
- Rack & pinion mechanism
- Food dispenser tray

### GPIO Pins (ALFRIDCL - Pi3B+)
- GPIO 17 = LEFT Limit Switch (NO/Cancel)
- GPIO 27 = RIGHT Limit Switch (YES/Dispense)
- GPIO 21 = Servo PWM Control (Rack & Pinion)

## Wiring Diagram

### Limit Switches
LEFT Limit Switch (NO/Cancel):
3.3V → Limit Switch → GPIO 17
GPIO 17 → 10kΩ resistor → GND
RIGHT Limit Switch (YES/Dispense):
3.3V → Limit Switch → GPIO 27
GPIO 27 → 10kΩ resistor → GND

### Servo Motor (Rack & Pinion)
Servo Control (PWM): GPIO 21
Servo Power: 5V
Servo Ground: GND

## Control Logic

### YES Button (RIGHT Limit Switch - GPIO 27)
- Triggered → Dispense food
- Servo moves forward via rack & pinion
- Tray extends to user
- After 2 seconds, servo retracts
- Tray returns to home position

### NO Button (LEFT Limit Switch - GPIO 17)
- Triggered → Cancel/reset
- Servo returns to home position
- Tray retracts immediately
- Ready for next user

## Servo Food Dispenser Node Code

### servo_food_dispenser_node.py
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
import RPi.GPIO as GPIO
import time

class ServoFoodDispenserNode(Node):
    def __init__(self):
        super().__init__('servo_food_dispenser')
        
        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Limit switch pins
        self.left_limit_pin = 17    # NO button
        self.right_limit_pin = 27   # YES button
        
        # Servo PWM pin
        self.servo_pin = 21
        
        # Setup pins
        GPIO.setup([self.left_limit_pin, self.right_limit_pin], GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.servo_pin, GPIO.OUT)
        
        # Setup PWM for servo (50Hz for standard servo)
        self.servo_pwm = GPIO.PWM(self.servo_pin, 50)
        self.servo_pwm.start(0)
        
        # Servo positions (duty cycle %)
        self.servo_home = 5.0        # Home position (0°)
        self.servo_extended = 10.0   # Extended position (90°)
        self.servo_fully_extended = 12.5  # Fully extended (180°)
        
        # Current servo position
        self.servo_position = self.servo_home
        
        # Setup event detection
        GPIO.add_event_detect(self.left_limit_pin, GPIO.RISING, callback=self.no_button_pressed, bouncetime=200)
        GPIO.add_event_detect(self.right_limit_pin, GPIO.RISING, callback=self.yes_button_pressed, bouncetime=200)
        
        # Publishers
        self.dispenser_pub = self.create_publisher(Bool, '/food_dispensed', 10)
        self.servo_status_pub = self.create_publisher(Float32, '/servo_position', 10)
        
        self.get_logger().info('Servo Food Dispenser Node started')
        self.move_servo(self.servo_home)
    
    def yes_button_pressed(self, channel):
        """YES button (RIGHT limit switch GPIO 27) - Dispense food"""
        self.get_logger().info('YES button pressed (GPIO 27) - Dispensing food!')
        
        # Move servo to extended position
        self.move_servo(self.servo_extended)
        time.sleep(1)
        
        # Move to fully extended
        self.move_servo(self.servo_fully_extended)
        
        # Publish dispensed message
        msg = Bool()
        msg.data = True
        self.dispenser_pub.publish(msg)
        
        # Keep extended for 2 seconds
        time.sleep(2.0)
        
        # Retract servo back to home
        self.move_servo(self.servo_extended)
        time.sleep(0.5)
        self.move_servo(self.servo_home)
        
        self.get_logger().info('Food dispensed - Tray retracted')
    
    def no_button_pressed(self, channel):
        """NO button (LEFT limit switch GPIO 17) - Cancel/Reset"""
        self.get_logger().info('NO button pressed (GPIO 17) - Cancelling!')
        
        # Immediately return servo to home
        self.move_servo(self.servo_home)
        
        # Publish cancelled message
        msg = Bool()
        msg.data = False
        self.dispenser_pub.publish(msg)
        
        self.get_logger().info('Tray retracted - Ready for next user')
    
    def move_servo(self, duty_cycle):
        """Move servo to specific duty cycle"""
        self.servo_pwm.ChangeDutyCycle(duty_cycle)
        self.servo_position = duty_cycle
        
        # Publish servo position
        pos_msg = Float32()
        pos_msg.data = duty_cycle
        self.servo_status_pub.publish(pos_msg)
        
        self.get_logger().debug(f'Servo moved to duty cycle: {duty_cycle}%')
    
    def destroy_node(self):
        self.servo_pwm.stop()
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ServoFoodDispenserNode()
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

## Test Servo & Buttons Script

### test_servo_dispenser.py
```python
#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

left_limit = 17
right_limit = 27
servo_pin = 21

GPIO.setup([left_limit, right_limit], GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(servo_pin, GPIO.OUT)

servo_pwm = GPIO.PWM(servo_pin, 50)
servo_pwm.start(0)

# Servo positions
servo_home = 5.0
servo_extended = 10.0
servo_fully_extended = 12.5

print("\n" + "="*60)
print("SERVO FOOD DISPENSER TEST")
print("="*60)
print("GPIO 17 (LEFT) = NO button (Cancel)")
print("GPIO 27 (RIGHT) = YES button (Dispense)")
print("GPIO 21 = Servo PWM (Rack & Pinion)")
print("\nPush limit switches to test servo movement")
print("Press Ctrl+C to stop\n")

def move_servo(duty_cycle):
    servo_pwm.ChangeDutyCycle(duty_cycle)
    print(f"Servo position: {duty_cycle}%")

try:
    # Test servo movement
    print("Testing servo movement...")
    
    print("\n1. Home position (0°)")
    move_servo(servo_home)
    time.sleep(1)
    
    print("2. Extended position (90°)")
    move_servo(servo_extended)
    time.sleep(1)
    
    print("3. Fully extended (180°)")
    move_servo(servo_fully_extended)
    time.sleep(1)
    
    print("4. Returning to home")
    move_servo(servo_extended)
    time.sleep(0.5)
    move_servo(servo_home)
    
    print("\nNow test with limit switches:")
    print("Push GPIO 27 (RIGHT) to dispense")
    print("Push GPIO 17 (LEFT) to cancel\n")
    
    while True:
        left_state = GPIO.input(left_limit)
        right_state = GPIO.input(right_limit)
        
        if right_state == GPIO.HIGH:
            print("\n[YES - GPIO 27] Dispensing food!")
            move_servo(servo_extended)
            time.sleep(1)
            move_servo(servo_fully_extended)
            time.sleep(2)
            move_servo(servo_extended)
            time.sleep(0.5)
            move_servo(servo_home)
            print("Food dispensed - Tray retracted\n")
            time.sleep(0.5)
        
        if left_state == GPIO.HIGH:
            print("\n[NO - GPIO 17] Cancelling!")
            move_servo(servo_home)
            print("Tray retracted\n")
            time.sleep(0.5)
        
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\n\nTest stopped!")
    move_servo(servo_home)
    servo_pwm.stop()
    GPIO.cleanup()
```

## Installation

### On ALFRIDCL (Pi3B+)
```bash
# Copy servo_food_dispenser_node.py to:
~/butler_ros2_ws/src/butler_gpio/butler_gpio/servo_food_dispenser_node.py

# Copy test_servo_dispenser.py to:
~/test_servo_dispenser.py

# Make executable
chmod +x ~/test_servo_dispenser.py

# Build
cd ~/butler_ros2_ws
colcon build

# Source setup
source /opt/ros/humble/setup.bash
source ~/butler_ros2_ws/install/setup.bash
```

## Launch Food Dispenser

### Terminal on ALFRIDCL
```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
ros2 run butler_gpio servo_food_dispenser_node
```

## Test Food Dispenser

### Manual Test
```bash
sudo python3 ~/test_servo_dispenser.py
# Push limit switches to test servo movement
```

### Check ROS2 Topics
```bash
# On ALFRIDROS (Pi5)
ros2 topic echo /food_dispensed
ros2 topic echo /servo_position

# Push limit switches on ALFRIDCL
# Should see servo move and topics update
```

## GPIO Pin Summary
GPIO 17 = LEFT Limit Switch (NO/Cancel)
GPIO 27 = RIGHT Limit Switch (YES/Dispense)
GPIO 21 = Servo PWM Control (Rack & Pinion)

## ROS2 Topics

### Publish
- `/food_dispensed` (std_msgs/msg/Bool) - Dispensing status
- `/servo_position` (std_msgs/msg/Float32) - Current servo duty cycle

## Safety Features

### Auto-Retract
```python
# Tray automatically retracts after 2 seconds
time.sleep(2.0)
self.move_servo(self.servo_home)
```

### Emergency Stop (NO button)
```python
# NO button immediately stops and retracts
self.move_servo(self.servo_home)
```

## Workflow

### User Interaction
1. User approaches ALFRID
2. User sees tray at home position
3. User presses GPIO 27 (YES) to request food
4. Servo extends tray with food via rack & pinion
5. User takes food from tray
6. After 2 seconds, tray retracts automatically
7. Ready for next user

### Cancel Workflow
1. User presses GPIO 17 (NO)
2. Tray immediately retracts
3. No food dispensed
4. Ready for next user

## Performance Notes
- Servo response time: ~50-100ms
- Food dispensing time: 2 seconds
- Retraction time: ~1 second
- Max dispense per hour: ~30 users (2 min per cycle)

## Servo Calibration

### Find Correct Duty Cycles
```python
# Standard servo: 1ms pulse = 0° (5% at 50Hz), 2ms = 180° (10%)
# Your servo may vary - test these values:

duty_cycles = {
    'home': 5.0,           # 0° - Home position
    'extended': 8.0,       # ~90° - Tray partially extended
    'fully_extended': 11.0 # ~180° - Tray fully extended
}

# Adjust based on your servo's movement
```

## Troubleshooting

### Servo Not Moving
```bash
# Check GPIO 21 is available
# Verify 5V power to servo
# Test with test_servo_dispenser.py
sudo python3 ~/test_servo_dispenser.py
```

### Limit Switches Not Triggering
```bash
# Test GPIO 17 and 27
sudo python3 -c "import RPi.GPIO as GPIO; GPIO.setmode(GPIO.BCM); GPIO.setup([17,27], GPIO.IN); print(GPIO.input(17), GPIO.input(27))"
```

### Servo Twitching
```python
# Add delay between servo movements
time.sleep(0.1)  # Small delay
```

### Rack & Pinion Misalignment
- Check gear mesh
- Lubricate with light grease
- Ensure rails are parallel
- Check for debris

## Advanced Features

### Multiple Food Options
```python
# Could add more servos for different food types
# YES button = Dispense food A
# Second button = Dispense food B
# etc.
```

### Food Quantity Control
```python
# Adjust duty cycle for different amounts
servo_small = 8.0      # Small portion (partial extension)
servo_medium = 10.0    # Medium portion
servo_large = 12.5     # Large portion (full extension)
```

## ALFRID Food Robot Features
- ✅ SLAM mapping with RPLidar
- ✅ Autonomous navigation
- ✅ Food dispensing via limit switch confirmation (GPIO 17/27)
- ✅ Servo-driven rack & pinion tray (GPIO 21)
- ✅ ROS2 networked architecture

## Future Enhancements
- [ ] Multiple food types (multiple servos)
- [ ] Quantity selector
- [ ] User feedback display
- [ ] Remote mobile app control
- [ ] Food inventory tracking
- [ ] Temperature-controlled tray

---

**ALFRID: The Ultimate Autonomous Food Serving Robot Butler!** 🤖🍽️

Property of 5KROBOTICS & MALHAR LABADE © 2025
