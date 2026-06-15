# Internet of Things Posture Corrector

An ESP32-based wearable posture correction device that uses an MPU-6050 accelerometer/gyroscope and an HC-SR04 ultrasonic sensor to detect and alert users when they have poor posture.

## Overview

This device monitors head tilt and front/back displacement in real-time and provides audio feedback via a piezo buzzer when poor posture is detected. It's ideal for desk workers, students, or anyone who needs posture correction throughout the day.

## Features

- **Head Tilt Detection**: Uses MPU-6050 IMU to detect pitch and roll angles
- **Front/Back Displacement**: Estimates posture displacement using accelerometer data
- **Ultrasonic Distance Monitoring**: HC-SR04 sensor tracks distance to detect forward lean
- **Audio Feedback**: Piezo buzzer alerts on detected poor posture
- **Recalibration Support**: GPIO button for on-the-fly recalibration without reboot

## Hardware Requirements

### Microcontroller
- **Freenove ESP32-S3-WROOM** (or compatible ESP32 board)

### Sensors
- **MPU-6050**: 6-axis accelerometer and gyroscope (I2C)
- **HC-SR04**: Ultrasonic distance sensor

### Actuators
- **Piezo Buzzer**: Audio alert (GPIO 10)

### Wiring Configuration

| Component | ESP32 Pin | Notes |
|-----------|-----------|-------|
| MPU-6050 SDA | GPIO 8 | I2C Data |
| MPU-6050 SCL | GPIO 9 | I2C Clock |
| Ultrasonic Trigger | GPIO 42 | HC-SR04 Trigger |
| Ultrasonic Echo | GPIO 41 | HC-SR04 Echo |
| Piezo Buzzer | GPIO 10 | Audio Alert |
| Recalibrate Button | GPIO 1 | Triggers recalibration (INPUT_PULLDOWN) |

## Dependencies

The project uses PlatformIO and requires the following libraries:

```
adafruit/Adafruit MPU6050
adafruit/Adafruit Unified Sensor
teckel12/NewPing@^1.9.7
```

These are automatically installed via `platformio.ini`.

## Software Architecture

### Source Files

- **`main.cpp`**: Core application loop and setup
  - Initializes sensors and pins
  - Monitors recalibration button
  - Reads sensor data and triggers alert logic

- **`head_tilt.cpp`**: MPU-6050 processing
  - Initializes the IMU
  - Calibrates pitch/roll offsets
  - Computes head tilt angles
  - Estimates front/back displacement

- **`ultrasonic.cpp`**: HC-SR04 distance measurement
  - Wraps NewPing library
  - Calibrates initial distance baseline
  - Validates distance readings
  - Applies threshold-based alerts

### Calibration Process

1. **MPU-6050 Calibration** (200 samples):
   - Captures baseline accelerometer/gyroscope offsets
   - Runs during setup and on recalibration

2. **Front/Back Calibration** (100 samples):
   - Establishes neutral posture displacement baseline
   - Prevents false positives from sensor drift

3. **Ultrasonic Calibration** (10 samples):
   - Measures initial distance at startup
   - Sets threshold (±7.5 cm default)
   - Alerts if object moves beyond threshold

## Building & Uploading

### Prerequisites

- [PlatformIO](https://platformio.org/install) installed
- Freenove ESP32-S3-WROOM connected to your computer

### Build

```bash
platformio run
```

### Upload to Device

```bash
platformio run --target upload
```

### Monitor Serial Output

```bash
platformio device monitor --speed 115200
```

## Usage

1. **Initial Setup**:
   - Connect the device and upload the firmware
   - Open the serial monitor at 115200 baud
   - The device will calibrate automatically during startup
   - You should see calibration messages confirming initialization

2. **Wearing the Device**:
   - Position the device on your head/neck
   - Ensure the ultrasonic sensor faces forward (toward your chest/desk)
   - Maintain a neutral posture during initial calibration

3. **Recalibration**:
   - Pull GPIO 1 HIGH (connect to 3.3V) to trigger recalibration without rebooting
   - The device will re-sample sensor baselines
   - A confirmation message will appear in the serial monitor

4. **Alert Feedback**:
   - Buzzer sounds at 1000 Hz when:
     - Head is tilted beyond threshold, OR
     - Front/back displacement exceeds 5 cm, OR
     - Ultrasonic distance exceeds ±7.5 cm threshold


## Configuration & Tuning

Edit values in the source files to customize behavior:

### In `main.cpp`
- `recalibratePin`: GPIO for recalibration button (default: 1)
- `piezoPin`: GPIO for buzzer (default: 10)
- `delay(100)`: Main loop delay in milliseconds

### In `ultrasonic.cpp`
- `triggerPin`: HC-SR04 trigger (default: 42)
- `echoPin`: HC-SR04 echo (default: 41)
- `maxDistance`: Max measurement range in cm (default: 400)
- `distanceThreshold`: Allowed deviation from baseline in cm (default: 7.5)

### In `head_tilt.cpp` (if accessible)
- Pitch/roll angle thresholds
- Acceleration-based displacement thresholds



