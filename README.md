# Project BALANCE: AI-Powered Self-Balancing Robot

[![C++](https://img.shields.io/badge/C%2B%2B-11-blue)](https://isocpp.org/)
[![Arduino](https://img.shields.io/badge/Arduino-ESP32-green)](https://www.arduino.cc/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

AI-powered self-balancing differential drive robot using edge computing, real-time control systems, and advanced optimization algorithms.

## Overview

Project BALANCE develops an autonomous self-balancing robot that uses AI-driven parameter tuning to achieve stable balancing on two wheels. The system combines:

- **PID Control**: Real-time balancing using proportional-integral-derivative control
- **Edge Computing**: ESP32 microcontroller for on-device processing
- **AI Optimization**: Automated PID parameter tuning using optimization algorithms
- **Sensor Fusion**: IMU (MPU6050) for orientation estimation

### Key Features

- Autonomous balancing with minimal manual tuning
- Real-time PID parameter optimization
- Edge AI deployment on resource-constrained hardware
- Configurable balance parameters via serial interface

## Architecture

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│   MPU6050 IMU   │────>│   ESP32 MCU     │────>│   Motor Driver  │
│  (Sensors)      │     │  (Edge AI)      │     │   (Actuators)   │
└─────────────────┘     └─────────────────┘     └─────────────────┘
                              │
                              v
                        ┌─────────────────┐
                        │   PID Controller │
                        │   + AI Tuning   │
                        └─────────────────┘
```

## Hardware Requirements

| Component | Specification |
|-----------|---------------|
| Microcontroller | ESP32 DevKit |
| IMU Sensor | MPU6050 (Gyroscope + Accelerometer) |
| Motor Driver | L298N or similar H-Bridge |
| Motors | 2x DC Gear Motors with encoders |
| Battery | 7.4V LiPo (2S) |
| Chassis | Custom 3D-printed differential drive |

## Installation

### Prerequisites

- Arduino IDE 2.0+ or PlatformIO
- ESP32 board package installed
- USB cable for flashing

### Setup

```bash
# Clone the repository
git clone https://github.com/dev-prashanna/Project_BALANCE.git
cd Project_BALANCE

# Open balance_1.ino in Arduino IDE
# Select board: ESP32 Dev Module
# Select correct COM port
# Upload sketch
```

### Arduino IDE Configuration

1. Install ESP32 board package:
   - File -> Preferences -> Additional Board Manager URLs:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
2. Tools -> Board -> Board Manager -> Search "esp32" -> Install

## Usage

### Basic Balancing

```cpp
#include <MPU6050.h>
#include <PID_v1.h>

// Initialize sensors and PID
MPU6050 mpu;
double setpoint = 0, input, output;
PID pid(&input, &output, &setpoint, 2.0, 0.5, 0.1, DIRECT);

void setup() {
    Serial.begin(115200);
    mpu.initialize();
    pid.SetMode(AUTOMATIC);
    pid.SetOutputLimits(-255, 255);
}

void loop() {
    // Read orientation
    input = getAngle();
    
    // Compute PID
    pid.Compute();
    
    // Drive motors
    driveMotors(output);
}
```

### AI Parameter Tuning

The robot uses a genetic algorithm to optimize PID parameters:

```cpp
// Define parameter ranges
float kp_range[] = {0.5, 5.0};
float ki_range[] = {0.0, 2.0};
float kd_range[] = {0.0, 3.0};

// Run optimization
optimizePID(kp_range, ki_range, kd_range, generations=50);
```

## Project Structure

```
Project_BALANCE/
├── balance_1.ino           # Main Arduino sketch
├── esp32_project/
│   ├── src/
│   │   ├── main.cpp       # Main application
│   │   ├── pid.cpp        # PID controller
│   │   ├── imu.cpp        # IMU interface
│   │   └── motor.cpp      # Motor control
│   └── platformio.ini     # PlatformIO config
├── docs/
│   └── schematic.pdf      # Circuit schematic
├── LICENSE                # MIT License
└── README.md              # This file
```

## PID Tuning

| Parameter | Initial Value | Optimized Value |
|-----------|---------------|-----------------|
| Kp (Proportional) | 2.0 | TBD |
| Ki (Integral) | 0.5 | TBD |
| Kd (Derivative) | 0.1 | TBD |

## Results

| Metric | Value |
|--------|-------|
| Balance Stability | TBD |
| Response Time | TBD |
| Max Tilt Angle | TBD |
| Battery Life | TBD |

## Future Work

- [ ] Implement Kalman filter for sensor fusion
- [ ] Add remote control via Bluetooth
- [ ] Implement machine learning for adaptive control
- [ ] Add obstacle avoidance
- [ ] Optimize power consumption
- [ ] Add web interface for parameter tuning

## References

- [PID Controller Theory](https://www.cds.caltech.edu/~murray/courses/cds101/fa02/caltech/astrom-ch6.pdf)
- [ESP32 Arduino Documentation](https://docs.espressif.com/projects/arduino-esp32/)
- [MPU6050 Datasheet](https://invensense.com/wp-content/uploads/2021/10/DS-MPU-6000-001-v1.7.pdf)
- [Self-Balancing Robot Projects](https://create.arduino.cc/projecthub/search?q=self+balancing+robot)

## License

This project is licensed under the MIT License -- see the [LICENSE](LICENSE) file for details.

## Author

**Prashanna Tiwari**
- GitHub: [@dev-prashanna](https://github.com/dev-prashanna)
- LinkedIn: [Prashanna Tiwari](https://www.linkedin.com/in/prashanna-tiwari-1b9a01398/)
