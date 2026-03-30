# Rover Gimbal Stabilization System

Firmware for a 2-axis camera stabilization system built for the SJSU Robotics
University Rover Challenge (URC) competition rover. The rover operates on
uneven terrain, so this system keeps the camera pointed forward regardless
of rover tilt.

## Overview

The hub board runs on an STM32 and is responsible for:
- Reading IMU data from an ICM20948 over I2C
- Running a complementary filter + PID loop to stabilize the camera pitch axis
- Receiving yaw/pitch target commands from mission control over CAN
- Replying with IMU telemetry (accel, gyro, magnetometer) on each heartbeat cycle

## How It Works

### Complementary Filter + PID
Raw accelerometer data is converted into a pitch angle. A complementary filter
fuses the accelerometer angle with gyroscope integration to reduce noise and
drift. A PID controller then drives the pitch servo to minimize the error
between the measured pitch and the target pitch.

### CAN Communication
The hub board communicates with mission control over a CAN bus at 100 kHz.
It listens for gimbal target requests (yaw/pitch axis + target angle) and
replies with raw IMU data on each heartbeat cycle.

### IMU Debugging
During development, the IMU exhibited unstable startup behavior. I2C traffic
was captured with a logic analyzer, which revealed a magnetometer reset issue
that was preventing stable sensor initialization. Fixing the reset sequence
restored reliable sensor output.

## Hardware
- STM32F103C8 microcontroller
- ICM20948 9-axis IMU (accel, gyro, magnetometer)
- 2x RC servos (900-2100 µs pulse range, 50 Hz)
- CAN transceiver

## Dependencies
Built with [libhal](https://github.com/libhal/libhal) using CMake and Conan.

## Build
```bash
conan build hub -pr stm32f103c8 -pr arm-gcc-14.2 -b missing
```

> **Note:** This repo contains the gimbal-relevant source files extracted from
> the larger hub-board codebase. The full rover firmware repository can be found
> at [urc-control-systems-2024](https://github.com/SJSURoboticsTeam/urc-control-systems-2024/tree/mast-gimbal/hub).
