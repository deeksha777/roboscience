import numpy as np


def diffdrive(x, y, theta, v_l, v_r, t, l):
    # straight line
    if (v_l == v_r):
        theta_n = theta
        x_n = x + v_l * t * np.cos(theta)
        y_n = y + v_l * t * np.sin(theta)
    # circular motion
    else:
        # Calculate the radius
        R = l / 2.0 * ((v_l + v_r) / (v_r - v_l))
        # computing center of curvature
        ICC_x = x - R * np.sin(theta)
        ICC_y = y + R * np.cos(theta)
        # compute the angular velocity
        omega = (v_r - v_l) / l
        # computing angle change
        dtheta = omega * t
        # forward kinematics for differential drive
        x_n = np.cos(dtheta) * (x - ICC_x) - np.sin(dtheta) * (y - ICC_y) + ICC_x
        y_n = np.sin(dtheta) * (x - ICC_x) + np.cos(dtheta) * (y - ICC_y) + ICC_y
        theta_n = theta + dtheta

    return x_n, y_n, theta_n
