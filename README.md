# Mobile Robot Localization using Extended Kalman Filter (EKF)
This project implements an Extended Kalman Filter (EKF) for mobile robot localization in a 2D space with noisy motion and GPS sensor data. The robot follows a nonlinear kinematic model and uses EKF to estimate its pose by fusing control inputs and noisy observations.

Key features include real-time simulation, visualization of uncertainty via covariance ellipses, and a clean, modular Python codebase. This project serves as a strong foundation for learning and extending EKF-based state estimation, sensor fusion, or even SLAM in probabilistic robotics.

# Project Overview
This Python-based project simulates a robot navigating in 2D space and estimates its position using:

- True state (ground truth)

- Dead reckoning (noisy control input)

- GPS measurements (noisy observations)

- Extended Kalman Filter (EKF) for sensor fusion and localization

The simulation shows how the EKF corrects noisy predictions using noisy measurements and produces a much more accurate estimate of the robot's path.

# How to Run
1) Install dependencies
   ```bash
   pip install -r requirements.txt
2) Run the simulation
   ```bash
   python main.py
You should see a live animated plot comparing the true, estimated, and dead reckoning paths.

# Features
- Full EKF Implementation: Includes nonlinear motion model, linearization via Jacobians, and full prediction-update loop with uncertainty propagation.

- Sensor & Control Noise Simulation: Simulates noisy GPS measurements and actuator uncertainties for realistic data fusion.

- Dead Reckoning vs EKF Comparison: Visualizes how EKF outperforms pure odometry using noisy inputs.

- Uncertainty Visualization: Draws dynamic 2D covariance ellipses to show evolving estimation confidence.

- Modular, Extensible Codebase: Clean separation of models, filter logic, and visualization—ideal for learning and experimentation.
   
# Future Work

- Use real sensor data (e.g., from a GPS module or IMU)

- Implement SLAM (Simultaneous Localization and Mapping)

- Replace motion model with differential drive

- Test different noise levels or maneuvers

References
Probabilistic Robotics – Thrun, Burgard, Fox

Python Robotics Resources - atsushisakai.github.io/PythonRobotics/
