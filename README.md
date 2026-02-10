# 5DOF Robotic Arm

A Python-based control system for a 5 Degrees of Freedom (DOF) robotic arm with inverse kinematics, real-time visualization, and Arduino integration.

## Overview

This project implements a complete software stack for controlling a 5DOF robotic arm, featuring:

- **Inverse Kinematics Engine**: Geometric and analytical IK solver for precise end-effector positioning
- **Real-time Visualization**: 3D matplotlib-based visualization of arm configuration and workspace
- **Arduino Integration**: Serial communication protocol for hardware control
- **Modular Architecture**: Separate modules for kinematics, visualization, mapping, and hardware interface

## Features

- Inverse kinematics computation for target pose calculation
- Forward kinematics validation
- Live 3D visualization of arm state
- Serial communication with Arduino-based servo controller
- Angle mapping and servo calibration utilities
- Workspace boundary detection

## System Architecture

```
┌─────────────┐
│   Main.py   │  ← Entry point
└──────┬──────┘
       │
       ├─────────────┬─────────────┬─────────────┬─────────────┐
       │             │             │             │             │
┌──────▼──────┐ ┌───▼────┐ ┌──────▼──────┐ ┌───▼────┐ ┌──────▼──────┐
│ Kinematics  │ │LiveViz │ │  ArduinoPy  │ │ Mapper │ │  ArmTest    │
│   .py       │ │  .py   │ │    .py      │ │  .py   │ │   .ino      │
└─────────────┘ └────────┘ └─────────────┘ └────────┘ └─────────────┘
     │                          │                             │
     │                          │                             │
  IK/FK Solver           Serial Comm                    Arduino FW
```

### Module Descriptions

| Module | Purpose |
|--------|---------|
| `Main.py` | Primary control loop and user interface |
| `Kinematics.py` | Forward and inverse kinematics algorithms |
| `LiveViz.py` | Real-time 3D visualization using matplotlib |
| `ArduinoPy.py` | Serial communication handler for Arduino |
| `Mapper.py` | Angle-to-servo position mapping and calibration |
| `ArmTest.ino` | Arduino firmware for servo control |

## Technical Details

### Kinematic Configuration

The robotic arm consists of 5 rotational joints:

1. **Base rotation** (θ₁): Rotation about vertical axis
2. **Shoulder** (θ₂): First arm segment elevation
3. **Elbow** (θ₃): Second arm segment elevation  
4. **Wrist pitch** (θ₄): End-effector pitch
5. **Wrist roll** (θ₅): End-effector roll

### Inverse Kinematics Approach

The IK solver implements a geometric decomposition method:

1. **Position decoupling**: Separate position and orientation problems
2. **2D projection**: Project 3D problem onto vertical plane containing target
3. **Analytical solution**: Solve planar 2R or 3R problem using trigonometry
4. **Orientation computation**: Calculate wrist angles from desired end-effector orientation

The solver handles:
- Multiple solutions (elbow-up/elbow-down configurations)
- Singularity detection
- Joint limit constraints
- Workspace boundary validation

### Coordinate Systems

```
Z (vertical)
│
│     Y
│    ╱
│   ╱
│  ╱
│ ╱
└──────────── X

Base frame: Right-handed coordinate system
Units: millimeters (position), degrees (angles)
```

### Communication Protocol

The Arduino interface uses a simple ASCII protocol over serial:

```
Format: "θ1,θ2,θ3,θ4,θ5\n"
Example: "90.0,45.5,30.2,60.0,0.0\n"

Baud rate: 9600 (configurable)
Terminator: Newline (\n)
```

## Installation

### Prerequisites

- Python 3.7+
- Arduino IDE (for uploading firmware)
- USB connection to Arduino board

### Python Dependencies

```bash
pip install numpy matplotlib pyserial
```

Or install from requirements file (if available):

```bash
pip install -r requirements.txt
```

### Arduino Setup

1. Open `ArmTest.ino` in Arduino IDE
2. Install required libraries:
   - Servo.h (typically included with Arduino IDE)
3. Configure servo pins in the sketch
4. Upload to your Arduino board

## Usage

### Basic Operation

1. **Configure hardware parameters** in respective modules:
   ```python
   # In Kinematics.py - set link lengths
   L1 = 100  # Base to shoulder (mm)
   L2 = 150  # Shoulder to elbow (mm)
   L3 = 120  # Elbow to wrist (mm)
   # etc.
   ```

2. **Set serial port** in `ArduinoPy.py`:
   ```python
   PORT = 'COM3'  # Windows
   # PORT = '/dev/ttyUSB0'  # Linux
   # PORT = '/dev/tty.usbserial-XXXX'  # macOS
   ```

3. **Run the main program**:
   ```bash
   python Main.py
   ```

### Example: Moving to Target Position

```python
from Kinematics import inverse_kinematics
from ArduinoPy import send_angles

# Define target position (x, y, z) in mm
target_position = [200, 100, 150]

# Solve IK
joint_angles = inverse_kinematics(target_position)

# Send to hardware
send_angles(joint_angles)
```

### Visualization

The `LiveViz.py` module provides real-time 3D rendering:

```python
from LiveViz import visualize_arm

# Visualize current configuration
joint_angles = [0, 45, 90, 30, 0]
visualize_arm(joint_angles)
```

## Configuration

### Servo Calibration

Edit `Mapper.py` to calibrate servo positions:

```python
# Map joint angle to servo PWM
SERVO_MIN = [500, 600, 550, 580, 600]   # Minimum pulse width
SERVO_MAX = [2500, 2400, 2450, 2420, 2400]  # Maximum pulse width
ANGLE_MIN = [-90, -90, -135, -90, -180]     # Minimum joint angle
ANGLE_MAX = [90, 90, 135, 90, 180]          # Maximum joint angle
```

### Joint Limits

Configure workspace constraints in `Kinematics.py`:

```python
JOINT_LIMITS = {
    'theta1': (-180, 180),
    'theta2': (-90, 90),
    'theta3': (-135, 135),
    'theta4': (-90, 90),
    'theta5': (-180, 180)
}
```

### Recommended Servos

- Base: High-torque servo (≥20 kg-cm)
- Shoulder/Elbow: Medium-torque servos (≥15 kg-cm)  
- Wrist: Low-torque servos (≥5 kg-cm)

## Future Development

### Upcoming 6DOF Version

Preview images show development of an upgraded 6DOF arm with spherical wrist:

**Version 1**: Non-spherical wrist design  
**Version 2**: Spherical wrist for full orientation control

Planned improvements:
- Additional wrist DOF for complete 6-axis control
- Enhanced kinematic solver with redundancy resolution
- Trajectory planning and motion control
- Force/torque sensing integration
- Computer vision for object detection and manipulation

## Troubleshooting

### Common Issues

**IK solver returns no solution**
- Verify target is within workspace
- Check joint limits configuration
- Ensure link lengths are correctly defined

**Servos not responding**
- Verify serial port and baud rate
- Check Arduino power supply
- Test individual servos with Arduino IDE examples

**Visualization doesn't update**
- Ensure matplotlib backend is correctly configured
- Check for blocking calls in main loop

**Jittery servo motion**
- Add smoothing/filtering to angle commands
- Implement trajectory interpolation
- Check for mechanical binding

## Acknowledgments

Development in progress. Additional documentation and examples will be added as the project evolves.

---

**Project Status**: Active Development  
**Last Updated**: February 2026

## 6-DOF Robotic-Arm CAD
Currently in works... :D
Here are some sneak peaks for the upcoming 6DoF Arm

**Version 1: (Non Spherical Wrist)

<img width="593" height="720" alt="image" src="https://github.com/user-attachments/assets/52f4986b-e1c8-430b-813c-43b63a6b1f17" />


**Version 2: (Spherical Wrist)

<img width="684" height="735" alt="image" src="https://github.com/user-attachments/assets/c8ab66b7-a530-4b54-80bc-d52bcb05d5a5" />

