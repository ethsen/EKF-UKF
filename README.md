# EKF-UKF

This repository contains tools for calibrating IMU sensors and estimating 3D orientation using an Unscented Kalman Filter (UKF) for my coursework in ESE 6500: Learning in Robotics. The implementation is based on the quaternion-based approach described in [A Quaternion-based Unscented Kalman Filter for Orientation Tracking by Edgar Kraft](https://natanaso.github.io/ece276a/ref/2_Kraft_UKF.pdf).

## Files Overview

### Calibration Utilities
- `calibrate_accelerometer.py`  
  Calibrates accelerometer bias and sensitivity using Vicon ground truth data through least squares optimization. Uses threshold filtering on acceleration components.

- `calibrate_gyroscope.py`  
  Computes gyroscope bias and sensitivity parameters by comparing IMU readings with angular velocities derived from Vicon rotation matrices. Implements weighted sensor fusion across multiple datasets.

### Core Implementation
- `estimate_rot.py`  
  **Quaternion-based UKF** implementation featuring:
  - Sigma point generation for nonlinear propagation
  - Gravity-aligned measurement updates
  - Euler angle conversion (roll/pitch/yaw)
  - Adaptive covariance handling

### Supplementary Code
- `ekf.py`  
  *Separate project:* Extended Kalman Filter example for parameter estimation in a synthetic dynamical system.


