# ALFRID Wiring Diagram

## L298N Motor Driver Connections

### Left Motor
- GPIO 27 → L298N IN1 (Forward)
- GPIO 17 → L298N IN2 (Backward)
- GPIO 12 → L298N ENA (PWM)

### Right Motor
- GPIO 25 → L298N IN3 (Forward)
- GPIO 5 → L298N IN4 (Backward)
- GPIO 18 → L298N ENB (PWM)

### Power
- Pi3B+ GND → L298N GND
- 12V Battery+ → L298N 12V+
- 12V Battery- → Pi3B+ GND (shared ground)

## Motor Connections
- LEFT_OUT1 & LEFT_OUT2 → Left Motor
- RIGHT_OUT3 & RIGHT_OUT4 → Right Motor

## Encoder Connections (Pi3B+ GPIO)

### Left Front Encoder
- GPIO 7 → A Channel
- GPIO 10 → B Channel

### Right Front Encoder
- GPIO 11 → A Channel
- GPIO 12 → B Channel

### Left Rear Encoder
- GPIO 4 → A Channel
- GPIO 8 → B Channel

### Right Rear Encoder (DISABLED)
- GPIO 15 → A Channel
- GPIO 14 → B Channel

## RPLidar A1 Connection

### USB Connection (Recommended)
- USB-C → Pi5 USB Port
- Device: /dev/ttyUSB0
- S/N: 9AC4FA86C2E392D0A5E59FF70D105C60

### Serial Connection (Alternative)
- TX → GPIO 15 (UART)
- RX → GPIO 14 (UART)

## Pin Summary

### Pi3B+ GPIO Usage
Motor Control:
GPIO 5, 12, 17, 18, 25, 27 (6 pins)
Encoders:
GPIO 4, 7, 8, 10, 11, 14, 15 (7 pins)

Property of 5KROBOTICS & MALHAR LABADE © 2025
