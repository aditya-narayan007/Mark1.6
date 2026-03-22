# Mark 1.6 — Mobile Manipulation Platform

Mark 1.6 is a wireless-controlled robotic platform combining a 4WD chassis with a 5-DOF 3D-printed robotic arm. It demonstrates coordinated locomotion and manipulation through a clean three-board architecture: a master controller, a dedicated chassis driver, and a handheld NRF transmitter.

---

## Table of Contents

- [System Overview](#system-overview)
- [Hardware Requirements](#hardware-requirements)
- [Architecture](#architecture)
- [Wiring](#wiring)
- [Firmware Files](#firmware-files)
- [Command Protocol](#command-protocol)
- [Arm Presets](#arm-presets)
- [Wireless Configuration](#wireless-configuration)
- [Library Dependencies](#library-dependencies)
- [Tuning & Calibration](#tuning--calibration)
- [Project Structure](#project-structure)
- [Usage](#usage)

---

## System Overview

```
[ Handheld Controller ]
  Arduino Uno
  NRF24L01+ (TX)
  2× Joystick + Pot + Buttons
        │
        │  NRF24L01+ (2.4 GHz)
        ▼
[ Master Controller ]                  [ Chassis Controller ]
  Arduino Mega                   ─────▶  Arduino Uno
  NRF24L01+ or HC-05 (BT)   Serial2     2× L298N motor drivers
  5× Servo (arm joints)                  4× DC gear motors
```

The master board is the single point of authority. All commands arrive at the master (via Bluetooth or NRF), which then drives the arm servos directly and forwards drive commands to the chassis board over UART.

---

## Hardware Requirements

### Per Board

| Board | Role | Recommended MCU |
|---|---|---|
| Master controller | Wireless RX, arm servos, command routing | Arduino Mega 2560 |
| Chassis controller | 4WD motor drive | Arduino Uno |
| Handheld transmitter | Joystick/button input, NRF TX | Arduino Uno |

### Sensors & Actuators

| Component | Quantity | Notes |
|---|---|---|
| MG996R or SG90 servo | 5 | Joints: base, shoulder, elbow, wrist, gripper |
| L298N dual H-bridge | 2 | One per side (left / right) |
| DC gear motor (6–12 V) | 4 | TT motors or N20 for compact builds |
| NRF24L01+ module | 2 | One TX (handheld), one RX (master) |
| HC-05 Bluetooth module | 1 | Optional — alternative to NRF on master |
| Analog joystick | 2 | KY-023 or equivalent |
| Potentiometer (10 kΩ) | 1 | Elbow angle control |
| Tactile push buttons | 6 | Gripper toggle + 5 preset buttons |
| 3D-printed arm structure | 1 | PLA or PETG, 5-link design |
| 4WD chassis frame | 1 | Standard acrylic or printed chassis |

### Power

| Rail | Voltage | Suggested Source |
|---|---|---|
| Motors | 7–12 V | 3S LiPo or 8× AA |
| Servos | 5–6 V | BEC or 4× AA |
| Arduino boards | 5 V (USB or VIN 7–12 V) | Shared with servo rail or separate USB |

> Always power the motor rail separately from the logic rail. Sharing a rail causes voltage dips that reset the Arduino mid-operation.

---

## Architecture

### Data flow

```
Joystick/Button input
        │
  mark1.6_transmitter.ino  (NRF TX, 20 Hz)
        │
        ▼  NRF24L01+ packet (32 bytes, ASCII command string)
        │
  mark1.6_master.ino
    ├── Arm commands  → servo.write() on pins 2–6
    └── Drive commands → Serial2 UART → mark1.6_chassis.ino
                                          └── L298N PWM output
```

### Board responsibilities

`mark1.6_master.ino` — receives wireless packets, parses the command protocol, sweeps arm servos smoothly, and forwards drive tokens to the chassis board. Also accepts direct USB serial input for bench testing.

`mark1.6_chassis.ino` — pure motor executor. Listens on `Serial` (connected to Master `Serial2`), interprets drive commands, and manages four independent motor channels with direction and PWM control. Includes a ramp function to smooth acceleration and prevent tip-over when the arm is extended.

`mark1.6_transmitter.ino` — handheld firmware. Reads two joysticks, one potentiometer, and six buttons at 20 Hz. Maps analog axes to joint increments and drive commands, then transmits via NRF24L01+.

---

## Wiring

### Master Controller (Arduino Mega)

#### Arm Servos

| Servo | Joint | Mega Pin |
|---|---|---|
| Servo 0 | Base rotation | 2 |
| Servo 1 | Shoulder | 3 |
| Servo 2 | Elbow | 4 |
| Servo 3 | Wrist | 5 |
| Servo 4 | Gripper | 6 |

All servo VCC → 5–6 V servo rail. All servo GND → common GND.

#### NRF24L01+ (when using `WIRELESS_NRF`)

| NRF Pin | Mega Pin |
|---|---|
| CE | 9 |
| CSN | 10 |
| MOSI | 51 |
| MISO | 50 |
| SCK | 52 |
| VCC | 3.3 V |
| GND | GND |

> NRF24L01+ runs at **3.3 V only**. Supplying 5 V will destroy the module. Add a 100 µF capacitor across VCC–GND to stabilise the supply.

#### HC-05 Bluetooth (when using `WIRELESS_BT`)

| HC-05 Pin | Mega Pin |
|---|---|
| TX | RX1 (19) |
| RX | TX1 (18) via 1 kΩ voltage divider |
| VCC | 5 V |
| GND | GND |

> HC-05 RX is 3.3 V tolerant on most modules, but a simple 1 kΩ / 2 kΩ divider from TX1 is recommended.

#### Chassis UART (to Chassis Uno)

| Mega Pin | Chassis Uno Pin |
|---|---|
| TX2 (16) | RX (0) |
| GND | GND |

---

### Chassis Controller (Arduino Uno)

#### L298N Driver A — Left motors

| L298N A Pin | Uno Pin |
|---|---|
| ENA (PWM) | 3 |
| IN1 | 22 |
| IN2 | 23 |
| ENB (PWM) | 5 |
| IN3 | 26 |
| IN4 | 27 |

#### L298N Driver B — Right motors

| L298N B Pin | Uno Pin |
|---|---|
| ENA (PWM) | 9 |
| IN1 | 24 |
| IN2 | 25 |
| ENB (PWM) | 11 |
| IN3 | 28 |
| IN4 | 29 |

> Both L298N modules share the motor supply rail. Connect their 12 V inputs together and to the battery. Connect all GND pins to a common ground with the Arduino GND.

---

### Handheld Transmitter (Arduino Uno)

#### Joysticks & Pot

| Input | Uno Pin |
|---|---|
| Left joystick X (turn) | A0 |
| Left joystick Y (fwd/back) | A1 |
| Right joystick X (arm base) | A2 |
| Right joystick Y (shoulder) | A3 |
| Elbow potentiometer | A4 |

#### Buttons (INPUT_PULLUP — connect to GND when pressed)

| Button | Uno Pin | Function |
|---|---|---|
| Gripper toggle | 2 | Open / close gripper |
| Preset 0 | 3 | Home |
| Preset 1 | 4 | Pick |
| Preset 2 | 5 | Place |
| Preset 3 | 6 | Carry |
| Preset 4 | 7 | Stow |

#### NRF24L01+ (same pinout as master, see above)

---

## Firmware Files

| File | Board | Description |
|---|---|---|
| `mark1.6_master.ino` | Arduino Mega | Wireless RX, arm control, command routing |
| `mark1.6_chassis.ino` | Arduino Uno | 4WD motor drive |
| `mark1.6_transmitter.ino` | Arduino Uno | Handheld joystick transmitter |

Upload each sketch to its designated board independently.

---

## Command Protocol

All commands are ASCII strings, newline-terminated (`\n`). Maximum length is 31 characters.

| Command | Format | Example | Description |
|---|---|---|---|
| Drive | `M <dir> <speed>` | `M F 180` | Direction: F B L R S X Y |
| Arm joint | `A <joint> <angle>` | `A 2 90` | Joint 0–4, angle in degrees |
| Gripper | `G <0\|1>` | `G 1` | 1 = close, 0 = open |
| Stop | `S` | `S` | Immediate stop, all motors |
| Preset | `P <id>` | `P 1` | Run preset pose 0–4 |

### Direction codes

| Code | Motion |
|---|---|
| F | Forward |
| B | Backward |
| L | Pivot left |
| R | Pivot right |
| S | Stop |
| X | Strafe left (mecanum only) |
| Y | Strafe right (mecanum only) |

### Joint index

| Index | Joint |
|---|---|
| 0 | Base rotation |
| 1 | Shoulder |
| 2 | Elbow |
| 3 | Wrist |
| 4 | Gripper |

---

## Arm Presets

Five built-in poses are defined in `mark1.6_master.ino`. All joints move simultaneously with linear interpolation over 40 steps (~600 ms).

| ID | Name | Base | Shoulder | Elbow | Wrist | Gripper |
|---|---|---|---|---|---|---|
| 0 | Home | 90 | 90 | 90 | 90 | 45 |
| 1 | Pick | 90 | 140 | 140 | 90 | 0 |
| 2 | Place | 45 | 60 | 60 | 90 | 90 |
| 3 | Carry | 90 | 70 | 90 | 90 | 90 |
| 4 | Stow | 90 | 160 | 160 | 45 | 90 |

To add custom presets, extend the `PRESETS[5][5]` array in `mark1.6_master.ino` and increment the first dimension.

---

## Wireless Configuration

### Switching between Bluetooth and NRF

At the top of `mark1.6_master.ino`, change the define:

```cpp
#define WIRELESS_BT    // Use HC-05 Bluetooth
// #define WIRELESS_NRF // Use NRF24L01+
```

Only one mode can be active at a time. The transmitter sketch always uses NRF — for Bluetooth control, use a phone app (e.g. Serial Bluetooth Terminal) sending the text command protocol over SPP.

### NRF channel address

The pipe address is `"MK16A"` on both master and transmitter. To run multiple Mark 1.6 units in the same space, change the address string in both files to a unique 5-character value.

### HC-05 baud rate

The default baud rate in the sketch is 9600. If your HC-05 is configured differently, update the `BT_SERIAL.begin()` call in `mark1.6_master.ino` to match.

---

## Library Dependencies

| Library | Used in | Install via |
|---|---|---|
| `Servo.h` | Master | Built into Arduino IDE |
| `SPI.h` | Master, Transmitter | Built into Arduino IDE |
| `RF24` by TMRh20 | Master, Transmitter | Arduino Library Manager |

Install RF24: **Sketch → Include Library → Manage Libraries → search "RF24" → install by TMRh20**.

---

## Tuning & Calibration

### Servo angle limits

Each joint has `ANGLE_MIN` and `ANGLE_MAX` defined in `mark1.6_master.ino`. Adjust these to match your printed arm's physical range of motion and prevent linkage collisions:

```cpp
const int ANGLE_MIN[5] = {  0,  20,  10,   0,   0 };
const int ANGLE_MAX[5] = {180, 160, 170, 180,  90 };
```

### Preset interpolation speed

The preset sweep runs 40 steps with 15 ms per step (~600 ms total). Increase the step delay for slower, more controlled motion:

```cpp
// In runPreset():
delay(15);  // Increase for slower movement
```

### Motor PWM range

Adjust in `mark1.6_chassis.ino`:

```cpp
const int MIN_PWM   = 60;    // Below this, motors stall
const int MAX_PWM   = 220;   // Leave headroom from 255 to protect motors
const int TURN_PWM  = 150;   // Pivot turn speed
```

### Joystick deadzone

In `mark1.6_transmitter.ino`, the deadzone prevents drift from imperfect joystick centering:

```cpp
const int DEADZONE = 50;   // Increase if joystick drifts at rest
```

### Transmission rate

The transmitter sends at 20 Hz by default. Lower `SEND_INTERVAL` for faster response; raise it to reduce radio traffic:

```cpp
const unsigned long SEND_INTERVAL = 50;  // ms (20 Hz)
```

---

## Project Structure

```
mark1.6/
├── mark1.6_master.ino         # Master controller firmware
├── mark1.6_chassis.ino        # 4WD chassis firmware
├── mark1.6_transmitter.ino    # Handheld NRF transmitter firmware
└── README.md                  # This file
```

---

## Usage

### First-time setup

1. Install the RF24 library via the Arduino Library Manager.
2. Upload `mark1.6_chassis.ino` to the chassis Uno first. Open Serial Monitor (9600 baud) and confirm `Mark 1.6 Chassis Ready` prints.
3. Upload `mark1.6_master.ino` to the Mega. Confirm `Mark 1.6 Master Ready` and the arm sweeps to the Home preset on boot.
4. Upload `mark1.6_transmitter.ino` to the handheld Uno.

### Testing without the transmitter

The master board echoes all commands on USB Serial at 9600 baud. Open Serial Monitor and type commands directly:

```
M F 160       → drive forward at PWM 160
A 1 120       → move shoulder to 120°
P 1           → run Pick preset
G 1           → close gripper
S             → stop
```

### Operating

1. Power the motor battery rail.
2. Power all three Arduino boards (USB or dedicated 5 V rail).
3. The master boots to the Home preset automatically.
4. Left joystick on the handheld drives the chassis; right joystick and potentiometer control the arm.
5. Preset buttons trigger smooth multi-joint moves.
6. The gripper button toggles open/closed.
