Robotics Demo
===
# 1. Kinematics (and Dynamics)
## Fn 1.1: Forward Kinematics and Inverse Kinematics (Symbolic) Calculation using DH Table
using symbolic math package in Python  - SymPy - to calculate forward and inverse kinematics with defined dh tables
- matlab code is also available in the repository, but requires symbolic math toolbox

## Fn 1.2: Forward Kinematics and Inverse Kinematics (Symbolic) Calculation in Joint Space

## Eg 1.2: Parrallel Robot

# 2. Visulization
using [yourdfpy](https://github.com/clemense/yourdfpy/tree/main) to load robot defined in [URDF format](http://wiki.ros.org/urdf/XML).
- "data/urdf_files": contains urdf files for different robots. [Source](https://github.com/ankurhanda/robot-assets)
## Fn 2.1: draw robot in 2D/3D space
based on yourdfpy, draw robot in 2D/3D space with matplotlib


# 3. Control Method
## Eg 3.1: Kalman Filter in Motion Tracking
with traditional Kalman Filter and Unscented Kalman Filter (filterpy package)
![Kalman Filter](imgs/KF_motion_estimation.gif)

## Eg 3.2: Unscented Kalman Filter with 1D Pendulum
- sigma points visualization
![Sigma Points](imgs/sigma_points.png){width=200 height=100}
- UKF estimation result (parameters sensitive)
![UKF](imgs/UKF_motion_estimation.gif)

## Eg 3.3: Particle Filter / Monte Carlo Localization in Motion Tracking
- Pariticle initialization
![Particle Initialization](imgs/particle_init.png){width=100 height=100}
- Particle Filter estimation result (parameters sensitive)
![Particle Filter](imgs/PF_estimation.gif)
