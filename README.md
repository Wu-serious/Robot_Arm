# Robot Arm

[中文版](./README_CN.md)

![Robot Arm](./images/3-1_系统实物正面照片.png)

## Overview

This project addresses the difficulty faced by people with mobility impairments in independently retrieving items from a desk. Built around **ESP32-P4** with a 4-DOF servo arm, it uses **smartphone IMU gesture control** ("tilt to move") via WiFi — no app required, just open a browser. An end-effector **MPU6050** forms a **PID closed-loop** that auto-corrects pitch in milliseconds. **Inverse kinematics** enables coordinate-driven one-touch grasping. Two modes (**gesture + coordinate**) complement each other in a **"gesture control + inverse kinematics + PID closed-loop"** architecture for assistive living.

## Quick Start

1. Install [ESP-IDF v5.5.4](https://docs.espressif.com/projects/esp-idf/en/v5.5.4/esp32p4/get-started/index.html)
2. Clone and configure:
   ```bash
   git clone <your-repo-url>
   cd Robot_Arm
   idf.py reconfigure
   ```
3. Obtain the C5 co-processor firmware (see [Co-processor Firmware](#co-processor-firmware) below), then build and flash:
   ```bash
   # First, set up the slave firmware (see instructions below)
   cd slave && idf.py set-target esp32c5 && idf.py reconfigure build flash && cd ..
   idf.py build flash monitor
   ```
4. Connect to WiFi `Arm_AP` (password: `12345678`), open `http://192.168.4.1`

> **Change credentials**: Modify `WIFI_SSID`/`WIFI_PASSWORD` in `main/main.c` or pass `-DWIFI_SSID=...` at compile time.

### System Architecture

![System Architecture](./images/2-1_系统整体架构图.svg)

- **Host (ESP32-P4)**: Main controller running the control logic, web server, and IMU feedback processing
- **Co-processor (ESP32-C5)**: Provides WiFi via SDIO connection (ESP-Hosted mode)
- **Smartphone IMU**: Sends phone attitude data via WebSocket as target input
- **MPU6050**: Mounted on the end-effector, measures actual attitude for PID closed-loop feedback
- **PCA9685**: I2C-based 16-channel 12-bit PWM driver for servo control (5 channels used)

### Control Modes

- **Mode 0 (Manual)**: Slider-based control via web interface
- **Mode 1 (Phone IMU + PID)**: Smartphone IMU gesture control with MPU6050 end-effector PID closed-loop correction

### Task Structure

| Task | Frequency | Responsibility |
|------|-----------|---------------|
| IMU_Feedback_Task | 100Hz | End-effector MPU6050 data collection, complementary filter fusion |
| Control_Task | 50Hz | PID closed-loop control + command generation, drives IMU mode |
| Servo_Task | 50Hz | Servo command execution, I2C lock coordination |
| WebStatus_Task | 20Hz | Web status publishing, WebSocket push |

![FreeRTOS Task Scheduling](./images/2-12_FreeRTOS四任务调度时序图.svg)

## Hardware

### Development Board

WTDKP4C5-S1-1V1 (ESP32-P4 + ESP32-C5)

### Servo Channels (5 channels)

| Channel | Function | PCA9685 Pin | Angle Range |
|---------|----------|-------------|-------------|
| 0 | Base rotation (J0) | PWM0 | 0-180° |
| 1 | Shoulder joint (J1) | PWM1 | 60-120° |
| 2 | Elbow joint (J2) | PWM2 | 40-140° |
| 3 | Wrist joint (J3) | PWM3 | 20-160° |
| 4 | Gripper | PWM4 | 0°(open) - 90°(closed) |

### Pin Mapping

#### ESP32-P4 Host

![SDIO Connection](./images/2-4_SDIO双芯通信连接图.svg)

| Pin | Function | Connected To |
|-----|----------|-------------|
| GPIO51 | I2C SDA | PCA9685 SDA, MPU6050 SDA |
| GPIO52 | I2C SCL | PCA9685 SCL, MPU6050 SCL |
| GPIO18 | SDIO CLK → C5 | ESP32-C5 CLK (GPIO9) |
| GPIO19 | SDIO CMD → C5 | ESP32-C5 CMD (GPIO10) |
| GPIO14 | SDIO D0 → C5 | ESP32-C5 D0 (GPIO8) |
| GPIO15 | SDIO D1 → C5 | ESP32-C5 D1 (GPIO7) |
| GPIO16 | SDIO D2 → C5 | ESP32-C5 D2 (GPIO14) |
| GPIO17 | SDIO D3 → C5 | ESP32-C5 D3 (GPIO13) |
| GPIO13 | SDIO Slave Reset → C5 | ESP32-C5 RST/EN |

#### ESP32-C5 Co-processor

SDIO pins are fixed by ESP-IDF for ESP32-C5 (cannot be reconfigured):

| Signal | C5 GPIO |
|--------|---------|
| CLK | GPIO9 |
| CMD | GPIO10 |
| D0 | GPIO8 |
| D1 | GPIO7 |
| D2 | GPIO14 |
| D3 | GPIO13 |

### I2C Configuration

![I2C Bus](./images/2-3_I2C总线连接示意图.svg)

- **Bus**: I2C_NUM_0
- **SDA**: GPIO51
- **SCL**: GPIO52
- **Frequency**: 100kHz
- **Devices**:
  - PCA9685 servo driver (I2C address: 0x40)
  - MPU6050 IMU (I2C address: 0x68, alt: 0x69)

### Bill of Materials

| Component | Quantity | Notes |
|-----------|----------|-------|
| WTDKP4C5-S1-1V1 board | 1 | ESP32-P4 + ESP32-C5 |
| Servo (MG996R, 180°) | 4 | J0=base, J1=shoulder, J2=elbow, J3=wrist |
| Servo (MG996R, gripper) | 1 | 0°–90° only (no full rotation needed) |
| PCA9685 module | 1 | I2C PWM driver, addr 0x40 |
| MPU6050 module | 1 | I2C IMU on end-effector, addr 0x68 |
| Jumper wires (Dupont) | several | For breadboard wiring |
| Breadboard (mini) | 1 | No soldering required |
| Micro USB cable | 1 | For P4 power and serial monitor |
| Power supply | 1 | 5V/2A+ recommended for servos |
| WiFi antenna | 1 | For C5 WiFi module |
| USB-to-serial programmer | 1 | For flashing C5 firmware |

## Software

### Build System

- **ESP-IDF**: v5.5.4
- **Target**: ESP32-P4 (Host) + ESP32-C5 (Co-processor)

![Software Architecture](./images/2-7_软件分层架构图.svg)

### Build & Flash

The board contains **two chips** connected via SDIO. The **C5 must be flashed first** (it provides WiFi to P4).

```bash
# Step 1: Obtain and flash ESP32-C5 co-processor firmware (WiFi chip)
# See [Co-processor Firmware](#co-processor-firmware) below for setup instructions
cd slave && idf.py set-target esp32c5 && idf.py reconfigure build flash && cd ..

# Step 2: Flash ESP32-P4 host firmware (main controller)
idf.py reconfigure build flash monitor
```

Or use the provided batch scripts:

```cmd
build_now.bat            # Build P4 firmware and flash it
```

### Co-processor Firmware

The ESP32-C5 co-processor firmware is the official [ESP-Hosted-MCU](https://github.com/espressif/esp-hosted-mcu) project maintained by Espressif. It is **not committed to this repository** — you must obtain it separately before building.

#### Setting up the Slave Firmware

```bash
# Clone the official ESP-Hosted-MCU repository
git clone https://github.com/espressif/esp-hosted-mcu.git

# Copy the slave firmware directory into this project
cp -r esp-hosted-mcu/slave .
cd slave
```

The slave firmware uses IDF Component Manager (see [`slave/main/idf_component.yml`](slave/main/idf_component.yml)). Dependencies (iperf, wifi-cmd, ping-cmd, mqtt) are downloaded automatically on first build — ensure internet connectivity during `idf.py reconfigure`.

> **Why flash C5 first?** The P4 host uses ESP-Hosted mode to access WiFi via the C5 over SDIO. Without C5 firmware, the P4 cannot initialize WiFi. The C5 is connected via fixed SDIO pins and must have valid firmware before P4 boots.

![Communication Protocol](./images/2-13_通信协议数据流图.svg)

### Project Structure

![Project Structure](./images/1-2_设计流程图.svg)

```
.
├── main/                      # Main application
│   ├── main.c                 # Application entry point, task definitions
├── components/
│   ├── Hardware/              # Hardware drivers
│   │   ├── I2C/               # I2C master driver
│   │   ├── PCA9685/           # Servo driver (PWM)
│   │   ├── MPU6050/           # IMU driver (sensor fusion)
│   │   └── Servo/             # Servo abstraction layer
│   ├── Middleware/            # Business logic
│   │   ├── Kinematics/        # Forward/inverse kinematics solver
│   │   └── Script/            # Scripted movements (move object, wave, reset)
│   └── Communication/         # Communication stack
│       ├── WiFi/              # WiFi AP setup
│       └── WebServer/         # HTTP + WebSocket server
└── slave/                     # ESP32-C5 co-processor firmware (not committed; see setup instructions above)
```

## Web Interface

Connect to the WiFi access point `Arm_AP` (password: `12345678`) and open `http://192.168.4.1` in a browser.

No app installation required — control the robot arm directly through the web page. The page displays real-time joint angles, IMU attitude, PID error, and other status information.

![Web Control Interface](./images/1-1_Web控制界面截图.png)

### Controls

| Command | Description |
|---------|-------------|
| `reset` | Reset all servos to 90° (neutral position) |
| `wave` | Waving animation (elbow J2 oscillates 3 times) |
| `mode` | Switch between Manual (0) and IMU+PID (1) |
| `servo0-4` | Set individual servo angle (0-180°) |
| `gripper_open` / `gripper_close` | Open/close gripper |
| `kinematic_move` | Move object with default coordinates |
| `kmove sx sy sz ex ey ez` | Move object with custom start/end coordinates |
| `phone_pitch` | Target pitch angle (IMU mode) |
| `phone_roll` | Target roll angle (IMU mode) |

## PID Closed-Loop Control

In Mode 1 (Phone IMU + PID):

- **Pitch**: Phone IMU target pitch → mapped to servo angles → MPU6050 end-effector pitch feedback → PID correction applied via Jacobian pseudo-inverse to J1/J2/J3
- **Roll**: Phone IMU roll → directly mapped to base rotation (J0), no PID needed (base rotation doesn't change MPU6050 roll)
- **Gripper**: Independent control via web commands

### PID Parameters

- **Kp**: 1.0
- **Ki**: 0.02
- **Kd**: 0.05
- **Integral limit**: 30.0
- **Output limit**: 15.0
- **Jacobian gain**: 0.8

### PID Design Highlights

![PID Algorithm Flow](./images/2-11_PID控制算法流程图.svg)

- **Anti-windup**: Integral term limited to ±30.0 to prevent overflow from sustained error
- **Derivative filtering**: Derivative term suppresses high-frequency noise, preventing servo jitter
- **Jacobian pseudo-inverse distribution**: End-effector pitch error is distributed to J1/J2/J3 proportional to each joint's partial derivative on pitch, rather than equal splitting
- **Smooth filtering**: Output passes through 0.8 coefficient exponential moving average filter to reduce servo shock
- **Deadband & rate limiting**: Commands with angle change <1° or interval <60ms are dropped, reducing I2C bus load

### I2C Bus Lock Coordination

All tasks (IMU_Feedback, Control, Servo) share the same I2C bus (SDA=GPIO51, SCL=GPIO52). A mutex semaphore coordinates access: IMU_Feedback_Task acquires the lock for reading MPU6050, Servo_Task acquires the lock for writing servo commands. If the lock is contended, commands queue rather than being dropped, and a 20ms timeout prevents lock starvation. This eliminates I2C collisions between concurrent IMU reads and servo writes.

### Delta-Limiter (Gyroscope Sign Flip Protection)

Phone gyroscopes exhibit a sign flip artifact near ±90° (-89° → +89°), causing the base angle to jump 178° in a single frame. The system implements a Δ-limiter that discards any frame where the roll change exceeds 170° — the angle is held at the previous frame's value. This prevents violent base spinning during normal IMU control.

### HTTP Fallback Mechanism

When the WebSocket connection is lost (e.g., browser tab backgrounded, WiFi hiccup), the web UI automatically falls back to HTTP GET requests (`/cmd?X_90`) for servo commands. The server supports the full command vocabulary via HTTP including `reset`, `wave`, `mode`, `gripper_open`, `gripper_close`, `kinematic_move`, `kmove`, and `phone_pitch`/`phone_roll`. This ensures control never breaks.

### Captive Portal DNS

An embedded DNS server on port 53 responds to all A-record queries with `192.168.4.1`. When a phone connects to `Arm_AP`, any browser URL entry (even `example.com`) redirects to the control page. No need to remember the IP address — just connect to WiFi and open any URL.

### Web UI — Three-Tab Design

The web interface is organized into three tabs:

![Servo Control Tab](./images/3-5_舵机控制Tab.png)

- **Servo Control**: Five slider channels (base/shoulder/elbow/wrist/gripper) with +/- buttons, quick-action buttons (reset, wave, gripper open/close)

![IMU + PID Tab](./images/3-6_体感PID_Tab.png)

- **IMU + PID**: Real-time pitch/roll display, target vs actual pitch, PID error/修正 values, sensor permission handling with DeviceOrientation API and DeviceMotion fallback

![Coordinate Pick & Place Tab](./images/3-7_坐标取放Tab.png)

- **Coordinate Pick & Place**: Start/end coordinate inputs (X/Y/Z) for inverse kinematics, execute pick-and-place with one tap, save/load coordinate presets to browser localStorage

### Robustness & Engineering Details

- **Exponential moving average filter** on all servo output (α=0.8) to smooth command transitions
- **Deadband mechanism**: Commands with angle change <1° or interval <60ms are silently dropped, reducing I2C bus load and servo wear
- **Δ-limiter**: Roll/ pitch frame changes >170° (gyro sign flip artifact) are discarded, holding the previous angle
- **I2C mutex with timeout**: Concurrent IMU reads and servo writes never collide; queued commands are executed in order when the bus is free
- **WebSocket auto-reconnect**: Exponential backoff (3s → max 30s), reconnects on tab restore/focus, heartbeat timeout (10s) disconnects stale clients
- **Captive Portal DNS**: All DNS queries resolve to `192.168.4.1`, so opening any URL after WiFi connection shows the control page
- **HTTP fallback**: When WebSocket is unavailable, all commands work via HTTP GET on `/cmd`, matching the ESP8266 command protocol

### Browser & Sensor Notes

- **Automatic redirect**: Connecting to `Arm_AP` and opening any URL auto-redirects to the control page via the built-in DNS server.
- **Phone IMU requires HTTPS or localhost**: The `DeviceOrientationEvent.requestPermission()` API is only available on HTTPS origins. Since the AP serves on `http://192.168.4.1`, iOS Safari (iPhone 13+) will block sensor access.
- **Recommended browser**: Use **Firefox** (install on your phone and open `http://192.168.4.1` directly). Firefox does not require permission prompts for device orientation and works reliably on both iOS and Android.
- **Alternative**: Chrome on Android works fine for IMU. On iOS, Firefox is the recommended choice.

## Kinematics

### Coordinate System

- Origin at base center (0, 0, 0)
- X-axis: forward, Y-axis: right, Z-axis: up
- All joint angles: 0-180°

### Arm Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| l1 | 78mm | Shoulder to elbow |
| l2 | 45mm | Elbow to wrist |
| l3 | 68mm | Wrist to arm end |
| gripper_length | 58mm | Arm end to gripper tip |
| base_height | 70mm | Base height |
| Max reach height | 319mm | base_height + l1 + l2 + l3 + gripper_length |

### Solvers

![Forward Kinematics](./images/2-8_正运动学求解流程图.svg)

- **Forward Kinematics**: Joint angles → end-effector position
- **Inverse Kinematics**: End-effector position → joint angles (geometric method with configuration preference)
  - **GRASP_A**: J2>90° (elbow forward), J3<90° (wrist forward), suitable for top-down grasping
  - **GRASP_B**: J2<90° (elbow backward), J3>90° (wrist backward), suitable for bottom-up grasping
  - **KINEMATIC_ANY**: Any valid solution, returns on first match

![Geometric IK](./images/2-9_几何法逆运动学求解流程图.svg)

- **Iterative IK**: Numerical Jacobian-based iterative solver for complex configurations (max_iter/tolerance configurable)

![Iterative IK](./images/2-10_迭代法逆运动学求解流程图.svg)

### Workspace

- Max reach radius: l1 + l2 + l3 + gripper_length = 249mm
- Max reach height: 319mm
- Coordinates outside the workspace are rejected with warning logs

## Troubleshooting

### 1. Servos Unresponsive / No Reaction

**Possible causes**:
- I2C bus communication failure (loose or reversed SDA/SCL wiring)
- PCA9685 not properly initialized (wrong I2C address)

**Troubleshooting steps**:
1. Check serial logs for `PCA9685_Init failed` or `Servo_SetAngle failed` errors
2. Verify SDA=GPIO51, SCL=GPIO52 wiring is correct
3. Measure I2C bus for pull-up resistors (typically 4.7kΩ-10kΩ)
4. Confirm PCA9685 AD0 pin is grounded (address 0x40), or connected to VCC (address 0x41)

### 2. MPU6050 Not Detected

**Possible causes**:
- AD0 pin level configuration incorrect
- I2C bus conflict (no conflict with PCA9685 as they share the bus with different addresses)

**Troubleshooting steps**:
1. Check serial logs: `MPU6050 not connected at 0x68, trying 0x69`
2. If 0x68 fails and auto-falls-back to 0x69, the AD0 pin is pulled high
3. Verify MPU6050 VCC/GND power supply (3.3V)
4. If both addresses fail, check I2C wiring

### 3. Base Spins Violently During IMU Control

**Possible causes**:
- Phone gyroscope sign flip near ±90° (-89° → +89°), causing base angle jump
- System has a built-in Δ-limiter (drops readings with >170° jump), but improper phone placement can still trigger it

**Solutions**:
1. Ensure phone is flat when roll ≈ 0° (horizontal placement)
2. In manual mode, use `servo0` slider to set base to a comfortable position first
3. Switch back to IMU mode and keep phone posture steady, avoid rapid large swings

### 4. Items Drop from Gripper During Grasp

**Possible causes**:
- Gripper closing force insufficient (servo angle hasn't reached 90°)
- End-effector attitude deviation causing item center-of-gravity shift
- PID closed-loop not active

**Troubleshooting steps**:
1. Use `gripper_close` command to verify full closure
2. Check PID logs `pid_pitch_error` and `pid_pitch_output` to confirm closed-loop is working
3. Verify item weight does not exceed gripper load capacity
4. Inspect gripper mechanical structure for wear or deformation

### 5. Inverse Kinematics Fails ("Unreachable")

**Possible causes**:
- Target coordinate exceeds workspace (height >319mm or horizontal distance >249mm)
- Target near singularity point (multiple joints collinear)

**Troubleshooting steps**:
1. Verify target coordinates are within reasonable ranges
2. Try `kmove` command with coordinates closer to the arm's current position
3. Logs print `Out of reach` or `No valid solution` to help locate the issue

### 6. Unstable WiFi Connection

**Possible causes**:
- ESP32-C5 co-processor firmware not loaded correctly
- SDIO communication link unstable

**Troubleshooting steps**:
1. Check serial logs for `WiFi AP started` message
2. Confirm C5 firmware is flashed (`idf.py set-target esp32c5 && idf.py build`)
3. Verify SDIO pin connections (GPIO13-19) are secure — poor contact causes WiFi disconnection
4. Try bringing the phone closer to the device

### 7. Build Failure

**Possible causes**:
- ESP-IDF version mismatch
- Component dependencies not installed

**Troubleshooting steps**:
1. Confirm ESP-IDF version is v5.5.4
2. Run `reconfigure.bat` to reinstall component dependencies
3. Clean old build artifacts and retry: `rm -rf build/ && idf.py reconfigure`

## License

This project is licensed under the [MIT License](./LICENSE).
