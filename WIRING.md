# ALFRIDCL (Pi3B+) Complete GPIO Pinout

## Motor Control Pins (CORRECTED)
LEFT MOTOR:
GPIO 5 = Forward (IN1)
GPIO 25 = Backward (IN2)
GPIO 6 = PWM Speed (ENA)
RIGHT MOTOR:
GPIO 23 = Forward (IN3)
GPIO 24 = Backward (IN4)
GPIO 22 = PWM Speed (ENB)

## Encoder Pins (2-encoder mode: RF + LR only)
RIGHT FRONT ENCODER:
GPIO 11 = A Channel
GPIO 12 = B Channel
LEFT REAR ENCODER:
GPIO 4  = A Channel
GPIO 8  = B Channel
DISABLED ENCODERS:
GPIO 7, 10 = Left Front (unused)
GPIO 15, 14 = Right Rear (hardware issue)

## Limit Switch Pins (Yes/No)
LEFT LIMIT SWITCH (NO/Cancel):
GPIO 17 = Input
RIGHT LIMIT SWITCH (YES/Dispense):
GPIO 27 = Input

## Servo Motor Pin
SERVO FOOD DISPENSER:
GPIO 13 = PWM Control

## Complete GPIO Usage Summary

| GPIO | Function | Type | Status |
|------|----------|------|--------|
| 4 | LR Encoder A | Input | ✅ Active |
| 5 | Left Motor FWD | Output | ✅ Active |
| 6 | Left Motor PWM (ENA) | Output | ✅ Active |
| 8 | LR Encoder B | Input | ✅ Active |
| 11 | RF Encoder A | Input | ✅ Active |
| 12 | RF Encoder B | Input | ✅ Active |
| 13 | Servo PWM | Output | ✅ Active |
| 20 | Left Limit Switch (NO) | Input | ✅ Active |
| 21 | Right Limit Switch (YES) | Input | ✅ Active |
| 22 | Right Motor PWM (ENB) | Output | ✅ Active |
| 23 | Right Motor FWD (IN3) | Output | ✅ Active |
| 24 | Right Motor BCK (IN4) | Output | ✅ Active |
| 25 | Left Motor BCK (IN2) | Output | ✅ Active |

## Available GPIO Pins (Still Free)
GPIO 2, 3, 7, 9, 10, 14, 15, 16, 17, 18, 19, 23, 26, 27

## Power Pins (Physical)
5V Power: Pins 2, 4
3.3V Power: Pins 1, 17
GND (Ground): Pins 6, 9, 14, 20, 25, 30, 34, 39

## L298N Motor Driver Connection (CORRECTED)
L298N PIN → Pi3B+ GPIO
IN1 (Left FWD) → GPIO 5
IN2 (Left BCK) → GPIO 25
ENA (Left PWM) → GPIO 6
IN3 (Right FWD) → GPIO 23
IN4 (Right BCK) → GPIO 24
ENB (Right PWM) → GPIO 22
GND → GND
12V → Battery 12V+

## Wiring Quick Reference

### Motors (L298N)
GPIO 5 (Left FWD)   → L298N IN1
GPIO 25 (Left BCK)  → L298N IN2
GPIO 6 (Left PWM)   → L298N ENA
GPIO 23 (Right FWD) → L298N IN3
GPIO 24 (Right BCK) → L298N IN4
GPIO 22 (Right PWM) → L298N ENB

### Encoders
GPIO 11 (RF A)  → Right Front Encoder A
GPIO 12 (RF B)  → Right Front Encoder B
GPIO 4  (LR A)  → Left Rear Encoder A
GPIO 8  (LR B)  → Left Rear Encoder B

### Limit Switches
GPIO 20 → Left Limit Switch (NO)
GPIO 21 → Right Limit Switch (YES)

### Servo
GPIO 13 → Servo PWM Signal
5V → Servo Power
GND → Servo Ground

## Pi3B+ Physical Pin Layout
 +3V3  1 ●●  2  +5V
 GPIO2  3 ●●  4  +5V
 GPIO3  5 ●●  6  GND
 GPIO4  7 ●●  8  GPIO14
  GND  9 ●● 10  GPIO15
GPIO17 11 ●● 12  GPIO18
GPIO27 13 ●● 14  GND
GPIO22 15 ●● 16  GPIO23
 +3V3 17 ●● 18  GPIO24
GPIO10 19 ●● 20  GND
 GPIO9 21 ●● 22  GPIO25
GPIO11 23 ●● 24  GPIO8
  GND 25 ●● 26  GPIO7
 GPIO0 27 ●● 28  GPIO1
 GPIO5 29 ●● 30  GND
 GPIO6 31 ●● 32  GPIO12
GPIO13 33 ●● 34  GND
GPIO19 35 ●● 36  GPIO16
GPIO26 37 ●● 38  GPIO20
  GND 39 ●● 40  GPIO21

## Current Pin Usage (13 GPIO pins)

Active:
- GPIO 4, 5, 6, 8, 11, 12, 13, 20, 21, 22, 23, 24, 25

Free for expansion:
- GPIO 2, 3, 7, 9, 10, 14, 15, 16, 17, 18, 19, 26, 27

## Pinout Conflicts
✅ **NO CONFLICTS** - All pins are unique and properly allocated!

## Quick Copy-Paste for Code
```python
# Motor pins (CORRECTED)
LEFT_FWD = 5
LEFT_BCK = 25
LEFT_PWM = 6

RIGHT_FWD = 23
RIGHT_BCK = 24
RIGHT_PWM = 22

# Encoder pins
RF_A = 11
RF_B = 12
LR_A = 4
LR_B = 8

# Limit switch pins
LEFT_LIMIT = 20    # NO button
RIGHT_LIMIT = 21   # YES button

# Servo pin
SERVO_PWM = 13
```

Property of 5KROBOTICS & MALHAR LABADE © 2025
