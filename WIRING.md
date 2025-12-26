# ALFRIDCL (Pi3B+) Complete GPIO Pinout

## Motor Control Pins
LEFT MOTOR:
GPIO 5 = Forward (IN1)
GPIO 25 = Backward (IN2)
GPIO 6 = PWM Speed (ENA)
RIGHT MOTOR:
GPIO 23 = Forward (IN3)
GPIO 24 = Backward (IN4)
GPIO 22 = PWM Speed (ENB)

## Encoder Pins (ALL 4 ENCODERS - ACTIVE)
RIGHT FRONT ENCODER:
GPIO 11 = A Channel
GPIO 12 = B Channel
LEFT REAR ENCODER:
GPIO 4  = A Channel
GPIO 8  = B Channel
LEFT FRONT ENCODER:
GPIO 7  = A Channel
GPIO 10 = B Channel
RIGHT REAR ENCODER:
GPIO 15 = A Channel
GPIO 14 = B Channel

## Limit Switch Pins (Yes/No)
LEFT LIMIT SWITCH (NO/Cancel):
GPIO 17 = Input
RIGHT LIMIT SWITCH (YES/Dispense):
GPIO 27 = Input

## Servo Motor Pin
SERVO FOOD DISPENSER:
GPIO 21 = PWM Control

## Complete GPIO Usage Summary

| GPIO | Function | Type | Status |
|------|----------|------|--------|
| 4 | LR Encoder A | Input | ✅ Active |
| 5 | Left Motor FWD | Output | ✅ Active |
| 6 | Left Motor PWM (ENA) | Output | ✅ Active |
| 7 | LF Encoder A | Input | ✅ Active |
| 8 | LR Encoder B | Input | ✅ Active |
| 10 | LF Encoder B | Input | ✅ Active |
| 11 | RF Encoder A | Input | ✅ Active |
| 12 | RF Encoder B | Input | ✅ Active |
| 14 | RR Encoder B | Input | ✅ Active |
| 15 | RR Encoder A | Input | ✅ Active |
| 17 | Left Limit Switch (NO) | Input | ✅ Active |
| 21 | Servo PWM | Output | ✅ Active |
| 22 | Right Motor PWM (ENB) | Output | ✅ Active |
| 23 | Right Motor FWD (IN3) | Output | ✅ Active |
| 24 | Right Motor BCK (IN4) | Output | ✅ Active |
| 25 | Left Motor BCK (IN2) | Output | ✅ Active |
| 27 | Right Limit Switch (YES) | Input | ✅ Active |

## Available GPIO Pins (Still Free)
GPIO 2, 3, 9, 13, 16, 18, 19, 20, 26

## L298N Motor Driver Connection
L298N PIN → Pi3B+ GPIO
IN1 (Left FWD) → GPIO 5
IN2 (Left BCK) → GPIO 25
ENA (Left PWM) → GPIO 6
IN3 (Right FWD) → GPIO 23
IN4 (Right BCK) → GPIO 24
ENB (Right PWM) → GPIO 22
GND → GND
12V → Battery 12V+

## Encoder Connections (ALL 4)
RIGHT FRONT:
GPIO 11 (A) → RF Encoder A
GPIO 12 (B) → RF Encoder B
LEFT REAR:
GPIO 4 (A)  → LR Encoder A
GPIO 8 (B)  → LR Encoder B
LEFT FRONT:
GPIO 7 (A)  → LF Encoder A
GPIO 10 (B) → LF Encoder B
RIGHT REAR:
GPIO 15 (A) → RR Encoder A
GPIO 14 (B) → RR Encoder B

## Limit Switches & Servo
GPIO 17 → Left Limit Switch (NO)
GPIO 27 → Right Limit Switch (YES)
GPIO 21 → Servo PWM (Rack & Pinion)

## Quick Copy-Paste for Code
```python
# Motor pins 
LEFT_FWD = 5
LEFT_BCK = 25
LEFT_PWM = 6

RIGHT_FWD = 23
RIGHT_BCK = 24
RIGHT_PWM = 22

# Encoder pins (ALL 4)
RF_A = 11
RF_B = 12
LR_A = 4
LR_B = 8
LF_A = 7
LF_B = 10
RR_A = 15
RR_B = 14

# Limit switch pins
LEFT_LIMIT = 17    # NO button
RIGHT_LIMIT = 27   # YES button

# Servo pin
SERVO_PWM = 21
```

Property of 5KROBOTICS & MALHAR LABADE © 2025
