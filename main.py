import numpy as np
import matplotlib.pyplot as plt

from models import calc_input, observation
from ekf import ekf_estimation
from utils.plot import plot_covariance_ellipse

# --- Simulation Parameters ---
DT = 0.1
SIM_TIME = 65.0
show_animation = True

# --- Noise Matrices ---
Q = np.diag([
    0.1,     # x
    0.1,     # y
    np.deg2rad(1.0),  # yaw
    1.0      # velocity
]) ** 2

R = np.diag([
    1.0,  # x
    1.0   # y
]) ** 2

INPUT_NOISE = np.diag([1.0, np.deg2rad(30.0)]) ** 2
GPS_NOISE = np.diag([0.5, 0.5]) ** 2

def main():
    print("EKF Mobile Robot Localization Simulation Start!")

    time = 0.0

    # Initial states
    xEst = np.zeros((4, 1))
    xTrue = np.zeros((4, 1))
    PEst = np.eye(4)
    xDR = np.zeros((4, 1))  # Dead reckoning

    # History arrays
    hxEst = xEst
    hxTrue = xTrue
    hxDR = xDR
    hz = np.zeros((2, 1))
    
    fig = plt.figure()
    plt.pause(2)
    
    while SIM_TIME >= time:
        time += DT

        u = calc_input()

        xTrue, z, xDR, ud = observation(xTrue, xDR, u, GPS_NOISE, INPUT_NOISE)

        xEst, PEst = ekf_estimation(xEst, PEst, z, ud, Q, R)

        # Store data
        hxEst = np.hstack((hxEst, xEst))
        hxDR = np.hstack((hxDR, xDR))
        hxTrue = np.hstack((hxTrue, xTrue))
        hz = np.hstack((hz, z))

        if show_animation:
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(hz[0, :], hz[1, :], ".g", label="GPS")
            plt.plot(hxTrue[0, :], hxTrue[1, :], "-b", label="True Path")
            plt.plot(hxDR[0, :], hxDR[1, :], "-k", label="Dead Reckoning")
            plt.plot(hxEst[0, :], hxEst[1, :], "-r", label="EKF Estimate")
            plot_covariance_ellipse(xEst[0, 0], xEst[1, 0], PEst)

            plt.axis("equal")
            plt.grid(True)
            plt.title("Extended Kalman Filter Localization")
            plt.xlabel("X [m]")
            plt.ylabel("Y [m]")
            plt.legend(loc="best")
            plt.pause(0.001)

    print("Simulation finished.")

if __name__ == '__main__':
    main()
