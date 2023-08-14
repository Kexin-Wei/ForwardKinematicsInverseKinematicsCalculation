# Robotics Demo
## Kinematics (and Dynamics)
### Fn 1: Forward Kinematics and Inverse Kinematics (Symbolic) Calculation
using symbolic math package in Python  - SymPy - to calculate forward and inverse kinematics with defined dh tables
- matlab code is also available in the repository, but requires symbolic math toolbox
### Fn 2: Parrallel Robot

## Visulization
### Fn 2: draw robot in 3D space (undone)
using matplotlib to draw robot in 3D space, can define color and link size
- compatible with Fn 1. forward kinematics and inverse kinematics

## Control Method
## Eg 1: Kalman Filter in Motion Tracking
with traditional Kalman Filter and Unscented Kalman Filter (filterpy package)
![Kalman Filter](imgs/KF_motion_estimation.gif)

## Eg 2: Unscented Kalman Filter with 1D Pendulum
- sigma points visualization
![Sigma Points](imgs/sigma_points.png){width=200 height=100}
- UKF estimation result (parameters sensitive)
![UKF](imgs/UKF_motion_estimation.gif)

## Eg 2: Particle Filter / Monte Carlo Localization in Motion Tracking
- Pariticle initialization
![Particle Initialization](imgs/particle_init.png){width=100 height=100}
- Particle Filter estimation result (parameters sensitive)
![Particle Filter](imgs/PF_estimation.gif)
