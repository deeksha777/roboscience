import numpy as np
import matplotlib.pyplot as plt
from PathTracking.forwardKinematics.differnceDrive import diffdrive

plt.gca().set_aspect('equal')
# set the distance between the wheels and the initial robot position
l = 0.5
ntheta = 0.0
x, y, theta = 1.5, 2.0, np.pi/2.0
# plot the starting position
plt.quiver(x, y, np.cos(theta), np.sin(theta))
print("starting pose: x: %f, y: %f, theta: %f" % (x, y, theta))
# first motion
v_l = 0.3
v_r = 0.3
t = 3
x, y, ntheta = diffdrive(x, y, theta, v_l, v_r, t, l)
plt.quiver(x, y, np.cos(ntheta), np.sin(ntheta))
print("after motion 1: x: %f, y: %f, theta: %f" % (x, y, ntheta))
# second motion
v_l = 0.1
v_r = -0.1
t = 2
x, y, ntheta = diffdrive(x, y, theta, v_l, v_r, t, l)
plt.quiver(x, y, np.cos(ntheta), np.sin(ntheta))
print("after motion 2: x: %f, y: %f, theta: %f" % (x, y, ntheta))
# third motion
v_l = 0.5
v_r = 0.0
t = 3
x, y, ntheta = diffdrive(x, y, theta, v_l, v_r, t, l)
plt.quiver(x, y, np.cos(ntheta), np.sin(ntheta))
print("after motion 3: x: %f, y: %f, theta: %f" % (x, y, ntheta))
plt.xlim([0.5, 2.5])
plt.ylim([1.5, 3.5])
plt.savefig("poses.png")
plt.show()

