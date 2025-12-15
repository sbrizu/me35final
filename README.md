# ME35 Final: Football — Multi-Robot Soccer System ⚽🤖

This repository contains the full software stack for our ME35 Final Project: a cooperative multi-robot system designed to score as many goals as possible within a five-minute match.

The system integrates a ball dispenser, a remotely controlled defensive robot, and an autonomous robot using AprilTag localization and computer vision.

---

## System Goal

The goal of this project is to design and implement a coordinated multi-robot system capable of repeatedly scoring goals under time constraints. A remotely controlled robot retrieves balls from a custom-built dispenser and delivers them to an autonomous robot. The autonomous robot then uses AprilTag-based localization and vision-based ball detection to navigate, align, and launch the ball into the goal.

---

## Team & Responsibilities

- **Sol Brizuela** — Ball dispenser mechanism and control code, goal detection
- **Natalie Dell’Immagine** — Remotely controlled defensive robot, BLE joystick control
- **Josh** — Goal detection system
- **Theo, Katie, Natalie** — Autonomous robot (vision, control, and system integration)

---

## System Architecture

### Subsystems
1. **Ball Dispenser**
   - Ultrasonic-triggered dual-servo door mechanism
   - Releases one ball per detection event

2. **Remotely Controlled Defensive Robot**
   - BLE joystick-based teleoperation
   - Differential drive + servo-controlled gate

3. **Autonomous Robot**
   - Computer vision (ball detection + AprilTag localization)
   - BLE motor control from laptop to ESP32
   - State-machine-based behavior

---

## Hardware Requirements

- **3 × ESP32 (MicroPython)**
  - Ball dispenser controller
  - RC defensive robot controller
  - Autonomous robot motor controller
- Grove ultrasonic distance sensor (single SIG pin)
- Servo motors (doors + gate)
- DC motors + motor drivers
- USB camera (computer vision)
- Laptop/PC for autonomous vision processing

---

## Software Requirements

### ESP32
- MicroPython firmware
- `BLE_CEEO` library

### Computer (Autonomous Vision)
- Python 3.x
- OpenCV
- NumPy
- `pupil_apriltags`
- `bleak`

Install dependencies:
```bash
pip install opencv-python numpy bleak pupil-apriltags
