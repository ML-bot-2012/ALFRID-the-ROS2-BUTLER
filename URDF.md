╔═══════════════╗
                    ║     LASER     ║  ← Red Cylinder (RPLidar)
                    ║    SCANNER    ║
                    ║   (0.06m r)   ║
                    ╚═══════════════╝
                           │
                ┌──────────┴──────────┐
                │                     │
          ╔═════════════════════════════════╗
          ║                                 ║
          ║      ALFRID CHASSIS             ║  ← Blue Box (30cm × 20cm × 15cm)
          ║      (Butler Robot)             ║
          ║                                 ║
          ╚═════════════════════════════════╝
          ▲           ▲           ▲           ▲
          │           │           │           │
        ╭─────╮   ╭─────╮   ╭─────╮   ╭─────╮
        │ ●●● │   │ ●●● │   │ ●●● │   │ ●●● │
        │ ENC │   │ ENC │   │ ENC │   │ ENC │
        │  LF │   │  RF │   │  LR │   │  RR │
        ╰─────╯   ╰─────╯   ╰─────╯   ╰─────╯
        │         │         │         │
      ╭─────╮ ╭─────────────────╮ ╭─────╮
      │MOTOR│ │                 │ │MOTOR│
      │  L  │ │   L298N DRIVER  │ │  R  │
      │(PWM)│ │  (2 Outputs)    │ │(PWM)│
      ╰─────╯ ╰─────────────────╯ ╰─────╯


        ╔════════════════════════════════════╗
        ║   ALFRID Robot Specifications     ║
        ╠════════════════════════════════════╣
        ║ Name: ALFRID (5KROBOTICS)         ║
        ║ Type: Butler Robot - Autonomous   ║
        ║                                    ║
        ║ CHASSIS:                           ║
        ║ • Blue Box: 30×20×15 cm            ║
        ║                                    ║
        ║ WHEELS (4x):                       ║
        ║ • LF (Left Front)    - 0.0325m r   ║
        ║ • RF (Right Front)   - 0.0325m r   ║
        ║ • LR (Left Rear)     - 0.0325m r   ║
        ║ • RR (Right Rear)    - 0.0325m r   ║
        ║                                    ║
        ║ ENCODERS (4x, 660 CPR):            ║
        ║ • RF (GPIO 11/12)                  ║
        ║ • LR (GPIO 4/8)                    ║
        ║ • LF (GPIO 7/10)                   ║
        ║ • RR (GPIO 15/14)                  ║
        ║                                    ║
        ║ MOTORS (2x DC):                    ║
        ║ • Left Motor  (PWM GPIO 6)         ║
        ║ • Right Motor (PWM GPIO 22)        ║
        ║                                    ║
        ║ MOTOR DRIVER:                      ║
        ║ • L298N with 2 outputs             ║
        ║ • Differential drive control       ║
        ║                                    ║
        ║ SENSORS:                           ║
        ║ • RPLidar A1 (Red, 0.06m r)        ║
        ║ • 4x Wheel Encoders (Quadrature)   ║
        ║                                    ║
        ║ NAVIGATION:                        ║
        ║ • ROS2 Jazzy (Pi5)                 ║
        ║ • ROS2 Humble (Pi3B+)              ║
        ║ • SLAM Toolbox                     ║
        ║ • Nav2 Stack                       ║
        ╚════════════════════════════════════╝
