# Self-Balancing Robot

## Overview

This project implements a real-time embedded control system for a two-wheeled, inverted pendulum robot. The core of the system is a closed-loop C++ pipeline running on an Arduino Uno that handles sensor fusion, state estimation, and motor control to maintain balance.

## Kalman Filter


## Prototype
![Demo](assets/Self-Balancing%20Robot%20Demo.gif)

## Architecture

Sensing via IMU -> State Estimation via Kalman filter -> Control (PID) -> Actuation Signal (Arduino PWM) -> Actuation (DC Motors and L298N Driver)

## Achievements

- **Real-Time Control Pipeline:** Developed and optimized a custom control loop, achieving a consistent 100 Hz (10ms latency) execution rate
- **Advanced State Estimation:** Engineered a custom Kalman Filter to fuse noisy MPU6050 accelerometer and gyro data, providing a robust tilt estimate by eliminating noise
- **Control System Tuning:** Implemented and tuned a PID controller, achieving stable self-balancing

## Components

- Arduino Uno R4 WiFi
- 2x TT DC Gear Motor
- MPU6050 IMU
- L298N Motor Driver

