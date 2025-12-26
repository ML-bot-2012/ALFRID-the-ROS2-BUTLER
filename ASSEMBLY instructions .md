# ALFRID Assembly Guide

## Safety First
- Wear safety glasses when handling tools
- Disconnect 12V battery before working on electronics
- Never force parts - if it doesn't fit, check alignment
- Keep small parts away from children and pets

## Tools Required
- Screwdriver set (Phillips & flat head)
- Wrench/socket set (M3, M4)
- Soldering iron 
- Multimeter for testing

## Fasteners & Materials
- M3 bolts and nuts
- Brass Standoff Set
- Double-sided tape or hot glue
- 40 Jumper wires (male-to-female)
- USB cables

---

## Part 1: Chassis Assembly

### Step 1.1: Prepare Base Plate
**Components:**
- [List components]
- 1 x Motor
- 1 x Motor Base
- 1 x Tire
- 1 x Coupling
- 1 x Hex Wrench
- 2 x Coupling Screw
- 6 x Small Screw
- 1 x Large Screw
- 3D printed base 

**Instructions:**


1.


<img width="675" height="583" alt="6AED5293-7CC7-4317-8657-ADA64E5EFEC3" src="https://github.com/user-attachments/assets/acfad927-a521-409d-84ab-02a36d5873ca" />



2.


<img width="1362" height="1302" alt="image" src="https://github.com/user-attachments/assets/36e53ca7-e0b5-4437-8a1f-76a1929a3763" />

3.


<img width="557" height="517" alt="Screenshot 2025-12-26 at 12 01 37 AM" src="https://github.com/user-attachments/assets/2915b2de-5c1a-4e7e-a3d9-fe289dafec87" />



**Tools Needed:**
- Phillips Head Screwdriver for Large and Small screws (Not INCLUDED with Motors) 
- Hexagonal Allen Key (INCLUDED with Motors)

---

## Part 2: Motor Driver & Power Installation

### Step 2.1: Mount L298N Motor Driver

**Components:**
- L298N Dual H-Bridge
- 2 M3 Phillips Head Screws
- 2 M3 Nuts
**Instructions:**
1.
<img width="416" height="397" alt="Screenshot 2025-12-26 at 12 27 16 AM" src="https://github.com/user-attachments/assets/8477c72a-f6eb-4a00-b1e9-ffd8c912cb51" />





**Tools Needed:**
- Phillips Head Screwdriver

---

### Step 2.2: Connect Motor Wires to L298N
**Components:**
- Motor wires (Male to Female)
- 

**Wiring:**
LEFT MOTOR → L298N OUT1 & OUT2
RIGHT MOTOR → L298N OUT3 & OUT4
## Motor Control Pins 
LEFT MOTOR:
GPIO 5 = Forward (IN1)
GPIO 25 = Backward (IN2)
GPIO 6 = PWM Speed (ENA)
RIGHT MOTOR:
GPIO 23 = Forward (IN3)
GPIO 24 = Backward (IN4)
GPIO 22 = PWM Speed (ENB)


**Instructions:**


1. Connect All motors to Pi 3B+ 


<img width="944" height="488" alt="image" src="https://github.com/user-attachments/assets/0185fda2-5e5e-445e-ac45-23e9717014e3" />


---

### Step 2.3: Install 12V Battery

**Components:**
-  KBT 12V Battery 70*55*20mm (L*W*T)
- Top Chassis frame (3D printed)

**Instructions:**


1.


<img width="653" height="594" alt="Screenshot 2025-12-26 at 12 44 27 AM" src="https://github.com/user-attachments/assets/d73033ae-5f9e-4a23-9287-062097b1c103" />



**⚠️ WARNING:** Always verify battery polarity before connecting!


---

### Step 3.1: Connect Encoder Wires
**Description:** [Your description here]
**Wiring:**
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
**Instructions:**
1.

<img width="972" height="1244" alt="image" src="https://github.com/user-attachments/assets/ffb08729-8977-49a9-a9f4-d6a11cd21145" />
Connect To PCB or Pi in general

<img width="1500" height="1500" alt="image" src="https://github.com/user-attachments/assets/19bd5fa6-b4f8-4ac1-ac18-8621ab20cf72" />

BLACK = ENCODER GND
BLUE = ENCODER POWER(3.3v)
RED = positive
WHITE = negative
GREEN = ENCODER B
YELLOW = ENCODER A
---

## Part 4: RPLidar Installation

### Step 4.1: Mount RPLidar Bracket
**Description:** [Your description here]
**Components:**
- RPLidar mounting bracket
- [List other components]

**Instructions:**
1. [Step 1]
2. [Step 2]
3. [Step 3]

**Tools Needed:**
- [Tool 1]
- [Tool 2]

**Time Estimate:** [X minutes]

---

### Step 4.2: Attach RPLidar A1
**Description:** [Your description here]
**Components:**
- RPLidar A1
- [List other components]

**Instructions:**
1. [Step 1]
2. [Step 2]
3. [Step 3]

**Tools Needed:**
- [Tool 1]
- [Tool 2]

**Time Estimate:** [X minutes]

---

### Step 4.3: Connect RPLidar USB
**Description:** [Your description here]
**Components:**
- USB-C cable
- [List other components]

**Instructions:**
1. [Step 1]
2. [Step 2]
3. [Step 3]

**Tools Needed:**
- [Tool 1]
- [Tool 2]

**Time Estimate:** [X minutes]

---

## Part 5: Electronics & Wiring

### Step 5.1: Mount Pi5 Case
**Description:** [Your description here]
**Components:**
- Pi5 protective case
- [List other components]

**Instructions:**
1. [Step 1]
2. [Step 2]
3. [Step 3]

**Tools Needed:**
- [Tool 1]
- [Tool 2]

**Time Estimate:** [X minutes]

---

### Step 5.2: Mount Pi3B+ Case
**Description:** [Your description here]
**Components:**
- Pi3B+ protective case
- [List other components]

**Instructions:**
1. [Step 1]
2. [Step 2]
3. [Step 3]

**Tools Needed:**
- [Tool 1]
- [Tool 2]

**Time Estimate:** [X minutes]

---

### Step 5.3: Connect Motor Control Wires (Pi3B+ to L298N)
**Description:** [Your description here]
**Wiring:**
GPIO 5 (Left FWD) → L298N IN1
GPIO 25 (Left BCK) → L298N IN2
GPIO 6 (Left PWM) → L298N ENA
GPIO 23 (Right FWD) → L298N IN3
GPIO 24 (Right BCK) → L298N IN4
GPIO 22 (Right PWM) → L298N ENB

**Instructions:**
1. [Step 1]
2. [Step 2]
3. [Step 3]

**Tools Needed:**
- [Tool 1]
- [Tool 2]

**Time Estimate:** [X minutes]

---

### Step 5.4: Connect Encoder Wires (Pi3B+)
**Description:** [Your description here]
**Wiring:**
RF A → GPIO 11
RF B → GPIO 12
LR A → GPIO 4
LR B → GPIO 8

**Instructions:**
1. [Step 1]
2. [Step 2]
3. [Step 3]

**Tools Needed:**
- [Tool 1]
- [Tool 2]

**Time Estimate:** [X minutes]

---

## Part 6: Limit Switches & Servo Installation

### Step 6.1: Mount Left Limit Switch
**Description:** [Your description here]
**Components:**
- Left limit switch
- 10kΩ pull-down resistor
- [List other components]

**Wiring:**
3.3V → Limit Switch → GPIO 17
GPIO 17 → 10kΩ resistor → GND

**Instructions:**
1. [Step 1]
2. [Step 2]
3. [Step 3]

**Tools Needed:**
- [Tool 1]
- [Tool 2]

**Time Estimate:** [X minutes]

---

### Step 6.2: Mount Right Limit Switch
**Description:** [Your description here]
**Components:**
- Right limit switch
- 10kΩ pull-down resistor
- [List other components]

**Wiring:**
3.3V → Limit Switch → GPIO 27
GPIO 27 → 10kΩ resistor → GND

**Instructions:**
1. [Step 1]
2. [Step 2]
3. [Step 3]

**Tools Needed:**
- [Tool 1]
- [Tool 2]

**Time Estimate:** [X minutes]

---

### Step 6.3: Build Rack & Pinion Mechanism
**Description:** [Your description here]
**Components:**
- Servo motor
- Pinion gear
- Rack gear
- [List other components]

**Instructions:**
1. [Step 1]
2. [Step 2]
3. [Step 3]

**Tools Needed:**
- [Tool 1]
- [Tool 2]

**Time Estimate:** [X minutes]

---

### Step 6.4: Install Tray Assembly
**Description:** [Your description here]
**Components:**
- Food dispenser tray
- Rails
- [List other components]

**Instructions:**
1. [Step 1]
2. [Step 2]
3. [Step 3]

**Tools Needed:**
- [Tool 1]
- [Tool 2]

**Time Estimate:** [X minutes]

---

### Step 6.5: Connect Servo to Pi3B+
**Description:** [Your description here]
**Wiring:**
GPIO 21 → Servo PWM Signal
5V → Servo Power
GND → Servo Ground

**Instructions:**
1. [Step 1]
2. [Step 2]
3. [Step 3]

**Tools Needed:**
- [Tool 1]
- [Tool 2]

**Time Estimate:** [X minutes]

---

## Part 7: Cable Management

### Step 7.1: Organize Motor Wires
**Description:** [Your description here]
**Components:**
- Zip ties
- Cable organizer
- [List other components]

**Instructions:**
1. [Step 1]
2. [Step 2]
3. [Step 3]

**Tools Needed:**
- [Tool 1]
- [Tool 2]

**Time Estimate:** [X minutes]

---

### Step 7.2: Route Power Cables
**Description:** [Your description here]
**Components:**
- [List components]

**Instructions:**
1. [Step 1]
2. [Step 2]
3. [Step 3]

**Tools Needed:**
- [Tool 1]
- [Tool 2]

**Time Estimate:** [X minutes]

---

### Step 7.3: Bundle GPIO Wires
**Description:** [Your description here]
**Components:**
- Jumper wires
- Zip ties
- [List other components]

**Instructions:**
1. [Step 1]
2. [Step 2]
3. [Step 3]

**Tools Needed:**
- [Tool 1]
- [Tool 2]

**Time Estimate:** [X minutes]

---

## Part 8: Final Testing & Calibration

### Step 8.1: Power On Test
**Description:** [Your description here]
**Components:**
- [List components]

**Instructions:**
1. [Step 1]
2. [Step 2]
3. [Step 3]

**Tools Needed:**
- Multimeter
- [Tool 2]

**Time Estimate:** [X minutes]

---

### Step 8.2: Motor Control Test
**Description:** [Your description here]
**Instructions:**
1. [Step 1]
2. [Step 2]
3. [Step 3]

**Tools Needed:**
- [Tool 1]
- [Tool 2]

**Time Estimate:** [X minutes]

---

### Step 8.3: Encoder Calibration
**Description:** [Your description here]
**Instructions:**
1. [Step 1]
2. [Step 2]
3. [Step 3]

**Tools Needed:**
- [Tool 1]
- [Tool 2]

**Time Estimate:** [X minutes]

---

### Step 8.4: Limit Switch Testing
**Description:** [Your description here]
**Instructions:**
1. [Step 1]
2. [Step 2]
3. [Step 3]

**Tools Needed:**
- [Tool 1]
- [Tool 2]

**Time Estimate:** [X minutes]

---

### Step 8.5: Servo & Tray Testing
**Description:** [Your description here]
**Instructions:**
1. [Step 1]
2. [Step 2]
3. [Step 3]

**Tools Needed:**
- [Tool 1]
- [Tool 2]

**Time Estimate:** [X minutes]

---

## Summary

### Total Assembly Time
- Estimated: [X hours]
- Actual: [Fill in after assembly]

### Parts Checklist
- [ ] Chassis assembled
- [ ] Motors installed
- [ ] Wheels attached
- [ ] Encoders installed
- [ ] RPLidar mounted
- [ ] L298N connected
- [ ] Battery installed
- [ ] Limit switches installed
- [ ] Servo & tray installed
- [ ] All wiring complete
- [ ] Cable management done
- [ ] All tests passed

### Known Issues
[Add any issues encountered during assembly]

### Notes
[Add any additional notes or modifications]

---

## Troubleshooting During Assembly

### Motor Won't Turn
- [Troubleshooting step 1]
- [Troubleshooting step 2]

### Encoder Not Reading
- [Troubleshooting step 1]
- [Troubleshooting step 2]

### Servo Not Moving
- [Troubleshooting step 1]
- [Troubleshooting step 2]

### Limit Switch Not Triggering
- [Troubleshooting step 1]
- [Troubleshooting step 2]

---

## Next Steps After Assembly
1. Verify all connections with multimeter
2. Test each component individually
3. Run integration tests (see QUICKSTART.md)
4. Calibrate motors and encoders
5. Test SLAM mapping
6. Test food dispensing system

---

Property of 5KROBOTICS & MALHAR LABADE © 2025
