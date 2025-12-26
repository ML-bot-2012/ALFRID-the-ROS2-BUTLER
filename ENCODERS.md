# Encoder Setup & Odometry Code

## Hardware
- 4x Encoder sensors (660 CPR each)
- 12V DC Encoder Gear Motors
- Pi3B+ GPIO pins for reading
- Currently using 2-encoder mode (RF + LR)

## Encoder Specifications
- Resolution: 660 counts per revolution
- Wheel Diameter: 0.065m (65mm)
- Distance per Count: 0.000309m
- Full Rotation: 360 degrees = 660 counts = 0.204m

## GPIO Pin Mapping (Pi3B+)

### Left Front Encoder (LF) - ACTIVE
- GPIO 7 = A Channel
- GPIO 10 = B Channel

### Right Front Encoder (RF) - ACTIVE
- GPIO 11 = A Channel
- GPIO 12 = B Channel

### Left Rear Encoder (LR) - ACTIVE
- GPIO 4 = A Channel
- GPIO 8 = B Channel

### Right Rear Encoder (RR) - DISABLED
- GPIO 15 = A Channel (hardware issue)
- GPIO 14 = B Channel (hardware issue)

## Encoder Odometry Node Code

### encoder_odometry_node.py
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
        
        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        self.wheel_diameter = 0.065
        self.encoder_resolution = 660
        self.wheel_base = 0.20
        self.meters_per_count = (math.pi * self.wheel_diameter) / self.encoder_resolution
        
        # Encoder pins for Pi 3B+ (2-encoder mode: RF and LR)
        self.rf_a_pin = 11
        self.rf_b_pin = 12
        self.lr_a_pin = 4
        self.lr_b_pin = 8
        
        GPIO.setup([self.rf_a_pin, self.rf_b_pin, self.lr_a_pin, self.lr_b_pin], GPIO.IN)
        
        self.rf_count = 0
        self.lr_count = 0
        
        self.last_rf_count = 0
        self.last_lr_count = 0
        
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        
        # Setup event detection
        GPIO.add_event_detect(self.rf_a_pin, GPIO.RISING, callback=self.rf_callback, bouncetime=5)
        GPIO.add_event_detect(self.lr_a_pin, GPIO.RISING, callback=self.lr_callback, bouncetime=5)
        
        self.odom_pub = self.create_publisher(Odometry, 'encoder_odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(0.05, self.odometry_callback)
        
        self.get_logger().info('Encoder Odometry Node started (Pi 3B+ GPIO, 2-encoder mode)')
    
    def rf_callback(self, channel):
        self.rf_count += 1
    
    def lr_callback(self, channel):
        self.lr_count += 1
    
    def odometry_callback(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt == 0:
            return
        
        rf_delta = self.rf_count - self.last_rf_count
        lr_delta = self.lr_count - self.last_lr_count
        
        rf_distance = rf_delta * self.meters_per_count
        lr_distance = lr_delta * self.meters_per_count
        
        right_distance = rf_distance
        left_distance = lr_distance
        distance = (left_distance + right_distance) / 2.0
        delta_theta = (right_distance - left_distance) / self.wheel_base
        
        self.theta += delta_theta
        self.x += distance * math.cos(self.theta)
        self.y += distance * math.sin(self.theta)
        
        linear_velocity = distance / dt if dt > 0 else 0.0
        angular_velocity = delta_theta / dt if dt > 0 else 0.0
        
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.angular.z = angular_velocity
        self.odom_pub.publish(odom)
        
        transform = TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation.z = math.sin(self.theta / 2.0)
        transform.transform.rotation.w = math.cos(self.theta / 2.0)
        self.tf_broadcaster.sendTransform(transform)
        
        self.last_rf_count = self.rf_count
        self.last_lr_count = self.lr_count
        self.last_time = current_time
    
    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdometryNode()
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

## Test Encoder Script

### test_encoders.py
```python
#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time
import math
import sys

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

encoders = {
    'LF': {'A': 7, 'B': 10, 'count': 0},
    'RF': {'A': 11, 'B': 12, 'count': 0},
    'LR': {'A': 4, 'B': 8, 'count': 0},
    'RR': {'A': 15, 'B': 14, 'count': 0},
}

for e in encoders.values():
    GPIO.setup([e['A'], e['B']], GPIO.IN)
    GPIO.add_event_detect(e['A'], GPIO.RISING, bouncetime=1)
    GPIO.add_event_detect(e['B'], GPIO.RISING, bouncetime=1)

wheel_diameter = 0.065
resolution = 660
meters_per_count = (math.pi * wheel_diameter) / resolution

print("\n" + "="*100)
print("ENCODER TEST - Counts, Distance & Angle")
print("="*100)
print("Rotate wheels by hand")
print("Press Ctrl+C to stop\n")

try:
    while True:
        print("\033[H\033[J")
        print("ENCODER DATA (Real-time)")
        print("-" * 100)
        print(f"{'Encoder':<10} {'Counts':<15} {'Distance (m)':<20} {'Angle (deg)':<20}")
        print("-" * 100)
        
        for name, pins in encoders.items():
            if GPIO.event_detected(pins['A']):
                pins['count'] += 1
            if GPIO.event_detected(pins['B']):
                pins['count'] += 1
            
            distance = pins['count'] * meters_per_count
            angle = (pins['count'] / resolution) * 360
            
            print(f"{name:<10} {pins['count']:<15d} {distance:<20.4f} {angle:<20.2f}")
        
        print("-" * 100)
        print("Full rotation = 660 counts = 360 degrees = 0.204m distance\n")
        time.sleep(0.5)

except KeyboardInterrupt:
    print("\n\nFinal Counts:")
    print("-" * 100)
    for name, pins in encoders.items():
        distance = pins['count'] * meters_per_count
        angle = (pins['count'] / resolution) * 360
        print(f"{name:<10} Counts: {pins['count']:<10d} Angle: {angle:.2f}° Distance: {distance:.4f}m")
    GPIO.cleanup()
    print("\nDone!")
```

## Installation

### On Pi3B+
```bash
# Copy encoder_odometry_node.py to:
~/butler_ros2_ws/src/butler_gpio/butler_gpio/encoder_odometry_node.py

# Copy test_encoders.py to:
~/test_encoders.py

# Make executable
chmod +x ~/test_encoders.py

# Build
cd ~/butler_ros2_ws
colcon build

# Source setup
source /opt/ros/humble/setup.bash
source ~/butler_ros2_ws/install/setup.bash
```

## Launch Encoder Odometry

### Terminal on Pi3B+
```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
ros2 run butler_gpio encoder_odometry_node
```

## Test Encoders

### Manual Test (Rotate wheels by hand)
```bash
sudo python3 ~/test_encoders.py
# Watch counts, distances, and angles update in real-time
```

### Check ROS2 Topic
```bash
# On Pi5
ros2 topic echo /encoder_odom
# Should show x, y, theta position updates
```

## Calibration

### Verify Encoder Resolution
```bash
# Rotate wheel exactly 1 full turn
# Expected count: ~660
# If different, adjust encoder_resolution in code
```

### Verify Wheel Diameter
```bash
# Measure wheel diameter physically
# Current: 0.065m (65mm)
# Update in code if different
```

### Verify Wheel Base
```bash
# Measure distance between left and right wheels
# Current: 0.20m (200mm)
# Update in code if different
```

## Troubleshooting

### No Encoder Counts
```bash
# Check GPIO connections
# Test with test_encoders.py script
# Verify GPIO pins are not in use by other processes

# Kill stuck processes
sudo pkill -9 python3
sudo reboot
```

### Inconsistent Counts
```bash
# Increase bouncetime in event detection
GPIO.add_event_detect(pin, GPIO.RISING, bouncetime=5)

# Or use polling instead of events
```

### Wrong Distance Calculation
```bash
# Verify wheel diameter is correct
# Check encoder resolution (should be 660)
# Calibrate by measuring physical rotation

# Adjust calculation:
meters_per_count = (math.pi * wheel_diameter) / encoder_resolution
```

### Position Drift
```bash
# Encoders drift due to:
# - Wheel slippage
# - Unequal wheel sizes
# - Dirty wheels

# Solutions:
# - Add IMU for orientation correction
# - Use SLAM for map-based correction
# - Increase wheel base measurement accuracy
```

## ROS2 Topics

### Publish
- `/encoder_odom` (nav_msgs/msg/Odometry) - Position and velocity
- `/tf` (tf2_msgs/msg/TFMessage) - Transform from odom to base_link

## Performance Notes
- Update Rate: 20 Hz
- Encoder Resolution: 660 CPR
- Drift: ~5% per meter (normal for wheel encoders)
- Best used with SLAM for correction

## Future Improvements
- Add 4-encoder averaging (currently 2-encoder mode)
- Implement IMU fusion for better angle estimation
- Add wheel slip detection
- Implement encoder health monitoring

Property of 5KROBOTICS & MALHAR LABADE © 2025
