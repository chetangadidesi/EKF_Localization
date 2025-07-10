import numpy as np
import math

DT = 0.1 # Discrete time step for motion updates

def calc_input():
    """
    Generate control input for the robot.
    This function simulates a constant velocity and constant yaw rate motion,
    which results in a circular trajectory.
    
    Returns:
        u (2x1 np.array): control input vector [v; yaw_rate]
    """
    v = 1.0 #[m/s]
    yawrate = 0.1 #[rad/s]
    u = np.array([[v], [yawrate]])
    return u

def observation(xTrue, xd, u, GPS_NOISE, INPUT_NOISE):
    """
    Simulate the system's response, GPS measurements, and noisy control inputs.

    Args:
        xTrue (4x1 np.array): True state of the robot
        xd (4x1 np.array): Dead reckoning state (estimated using noisy control)
        u (2x1 np.array): True control input
        GPS_NOISE (2x2 np.array): Measurement noise covariance matrix
        INPUT_NOISE (2x2 np.array): Input noise covariance matrix

    Returns:
        xTrue: Updated true state after applying motion model
        z: Noisy GPS observation of position [x; y]
        xd: Updated dead reckoning state using noisy control
        ud: Noisy control input
    """
    xTrue = motion_model(xTrue, u)
    
    # add noise to gps x-y
    z = observation_model(xTrue) + GPS_NOISE @ np.random.randn(2,1)
    
    # add noise to input
    ud = u + INPUT_NOISE @ np.random.randn(2,1)
    xd = motion_model(xd, ud)
    
    return xTrue, z, xd, ud

def motion_model(x, u):
    """
    Applies the motion model to compute the next state.

    State vector x = [x_position, y_position, yaw, velocity]
    Control input u = [velocity, yaw_rate]

    The model assumes a bicycle-like kinematic model:
        x_{t+1} = x_t + v*dt*cos(yaw)
        y_{t+1} = y_t + v*dt*sin(yaw)
        yaw_{t+1} = yaw_t + yaw_rate*dt
        v_{t+1} = v_t (assumed constant)

    Args:
        x (4x1 np.array): Current state
        u (2x1 np.array): Control input

    Returns:
        x (4x1 np.array): Predicted next state
    """
    F = np.array([[1.0, 0, 0, 0],
                  [0, 1.0, 0, 0],
                  [0, 0, 1.0, 0],
                  [0, 0, 0, 0]])
    
    B = np.array([[DT * math.cos(x[2, 0]), 0],
                  [DT * math.sin(x[2, 0]), 0],
                  [0.0, DT],
                  [1.0, 0.0]])
    
    x = F @ x + B @ u
    
    return x

def observation_model(x):
    """
    Observation model maps the full state to observable output.

    For this case, the robot's sensors (e.g., GPS) only observe position (x, y).

    Args:
        x (4x1 np.array): Current state

    Returns:
        z (2x1 np.array): Observed state [x; y]
    """
    H = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ])
    
    z = H @ x
    
    return z

def jacob_f(x, u):
    """
    Compute the Jacobian of the motion model (used in EKF prediction step).

    This linearizes the motion model around the current state.

    Args:
        x (4x1 np.array): Current state
        u (2x1 np.array): Control input

    Returns:
        jF (4x4 np.array): Jacobian matrix of the motion model w.r.t. state
    """
    yaw = x[2, 0]
    v = u[0, 0]
    jF = np.array([
        [1.0, 0.0, -DT * v * math.sin(yaw), DT * math.cos(yaw)],
        [0.0, 1.0, DT * v * math.cos(yaw), DT * math.sin(yaw)],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ])
    
    return jF

def jacob_h():
    """
    Jacobian of the observation model.

    Since observation is linear (just extracting x and y), the Jacobian is constant.

    Returns:
        jH (2x4 np.array): Jacobian of observation model
    """
    jH = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ])
    
    return jH