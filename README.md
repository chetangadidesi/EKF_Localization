# Mobile Robot Localization using Extended Kalman Filter (EKF)
This project demonstrates how to localize a mobile robot in 2D using the Extended Kalman Filter (EKF). The simulation combines motion modeling, sensor fusion (GPS + odometry), and noisy control input to estimate the robot's trajectory in real-time.

# Project Overview
This Python-based project simulates a robot navigating in 2D space and estimates its position using:

- True state (ground truth)

- Dead reckoning (noisy control input)

- GPS measurements (noisy observations)

- Extended Kalman Filter (EKF) for sensor fusion and localization

The simulation shows how the EKF corrects noisy predictions using noisy measurements and produces a much more accurate estimate of the robot's path.

# Project Structure
<details> <summary>📁 <code>Project Structure</code></summary>
mobile_robot_ekf_project/
│
├── main.py                   # Main simulation loop and visualization
├── ekf.py                    # EKF prediction and update functions
├── models.py                 # Motion model, observation model, and Jacobians
├── utils/
│   └── plot.py               # Utility for drawing covariance ellipses
├── README.md                 # Project documentation (this file)
├── requirements.txt          # Required Python packages
</details>

# How to Run
1) Install dependencies
   ```bash
   pip install -r requirements.txt
2) Run the simulation
   ```bash
   python main.py
You should see a live animated plot comparing the true, estimated, and dead reckoning paths.

# Features
✅ Extended Kalman Filter (EKF) implemented from scratch
Includes both prediction and correction steps with motion and observation model linearization using Jacobians.

✅ Nonlinear kinematic motion model
Simulates a realistic bicycle-like robot model with position, orientation, and velocity.

✅ Noisy control input simulation
Emulates real-world motor uncertainty by adding Gaussian noise to velocity and yaw rate commands.

✅ GPS-like noisy observation model
Uses simulated noisy measurements to represent imperfect real-world sensors.

✅ Covariance propagation and Kalman gain computation
Proper uncertainty handling using prediction and innovation covariance matrices (P, S).

✅ Real-time animation with uncertainty ellipses
Visualizes evolving state estimates along with uncertainty using 2D Gaussian ellipses.

✅ Dead reckoning vs. EKF comparison
Highlights the benefits of sensor fusion by comparing pure odometry with EKF estimates.

✅ Modular code structure for reusability
Separated into motion/observation models, EKF logic, and plotting utilities for clarity and extension.

✅ State and measurement history tracking
Maintains trajectory data for analysis, plotting, or post-processing.

✅ Support for batch simulation
Easily adaptable for offline data simulation or log replay scenarios.
   
# Future Work

- Use real sensor data (e.g., from a GPS module or IMU)

- Implement SLAM (Simultaneous Localization and Mapping)

- Replace motion model with differential drive

- Test different noise levels or maneuvers

References
Probabilistic Robotics – Thrun, Burgard, Fox

Python Robotics Resources - atsushisakai.github.io/PythonRobotics/
