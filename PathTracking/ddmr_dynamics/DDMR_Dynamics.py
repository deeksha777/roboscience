import math
import numpy as np
import matplotlib.pyplot as plt
# Differential drive robot, Controls based on Kanayama [6]:
# 2 drive wheels and a castor, Lyapunov Stable
# Tracking a 0.6 m radius, 12 second circle
# Iterative solution of state equations
# Scott R. Norr

# Set up iteration counter, timestep (in seconds), real
# time counter (seconds)
N = 12000
i = 0
dt = 0.0001
t = np.zeros(N)

# Robotic Platform Constants
# Wheel radii and robot half-dimensions in meters
r1 = 0.0322
r2 = 0.0322
l = 0.064
x = 0.05
d = 0.01
m = 0.405

# Controller Gain Constants
kx = 1.1
ky = 6
kw = 8
vref = 0.3
wref = 0.3

# Reference is the Global Frame at [0,0,0],
# Initial position of tracked object is G
Ref = np.array([[0], [0], [0]])
G = np.zeros([3, N])
G[:, 0] = np.array([0.6, 0, math.pi / 2])

# Initial position vector of Robot, Ei, in the global frame
Ei = np.zeros([3, N])
# Calculate initial error vector:
e = np.subtract(G, Ei)
# A-matrix for differential drive, where
# wheel velocities,[phi]=inv(A)*[v;w]
A = np.array([[r1 / 2, r2 / 2], [r1 / (np.dot(2, l)), - r2 / (np.dot(2, l))]])
Ainv = np.linalg.inv(A)

# Inverse of the Jacobian matrix of rolling and sliding
# constraints on robot motion:
# Where Jac =[Vx, 0, W]
Jinv = np.array([[0.5, 0.5, 0], [0, 0, 1], [1 / (np.dot(2, l)), - 1 / (np.dot(2, l)), 0]])

# initialize other state variables, robot velocity
# and rate of rotation wrt global frame
v = np.zeros(N)
w = np.zeros(N)
vdot = np.array(0)
wdot = np.array(0)
u = np.array([[0], [0]])
# Phi are the desired wheel velocities, (rad/sec)
phi = np.zeros([2, N])

# Motor model initialize (GR is gear ratio of motors),
# two drive motors (each slightly different)
# Jm is effective inertial moment of robot,
# B1,2 dynamic friction
Ra = np.array([[5.32], [5.31]])
La = 0.0023
Ka = np.array([[0.0054], [0.0054]])
Kt = Ka
GR = 48
Jm = 0.0001 / GR
# B1 = np.dot(0.07, 1j)
B1 = (0 + 0.07j)
print(B1)
B2 = B1
La1 = np.array([[0.0023], [0.0026]])

# motor variables, VDC1, VDC2 are the voltages
# applied to each motor thru a motor controller,
# Ia are the armature currents, omega are the
# shaft velocities of the motor. GR will change
# shaft vel. to wheel vel.
Vdc = np.zeros([2, N])
Omega = np.zeros([2, N])
Ia = np.zeros([2, N])
Iabs = np.zeros([2, N])
E = np.zeros(1)
E[0] = 0
Vbatt = np.zeros(N)
Vbatt[0] = 6.1
AH = np.zeros(N)
AH[0] = 2.5
Rbatt = 0.037
# tau = np.array([[0], [0]])
# Ides = np.array([[0], [0]])

# While loop to calculate state variables on an
# iterative basis
count = N - 1
while i < count:
    # Update the real time counter
    t[i + 1] = t[i] + dt
    # Generate line to track: 1) .6m radius circle in 12sec
    a = np.dot(dt, np.array([[np.dot(np.dot(-0.6, np.pi), math.sin(np.dot(np.pi, t[i + 1]) / 6)) / 6],
                             [np.dot(np.dot(0.6, np.pi), math.cos(np.dot(np.pi, t[i + 1]) / 6)) / 6], [np.pi / 6]]))
    G[:, [i + 1]] = G[:, [i]] + a

    # calculate new error in Robot Frame of reference

    err = np.dot(
        [[math.cos(Ei[2, i]), math.sin(Ei[2, i]), 0], [- math.sin(Ei[2, i]), math.cos(Ei[2, i]), 0], [0, 0, 1]],
        (G[:, [i + 1]] - Ei[:, [i]]))
    e[:, [i + 1]] = err

    # control law for velocity and rate of rotation
    # about robot center,(Kanayama)
    v[i + 1] = np.dot(vref, math.cos(e[2, [i + 1]])) + np.dot(kx, e[0, [i + 1]])
    w[i + 1] = wref + np.dot(ky, e[1, [i + 1]]) + np.dot(np.dot(kw, vref), math.sin(e[2, [i + 1]]))

    # Desired angular velocities at the Wheels, phi-1 and phi-2:
    phi[:, [i + 1]] = np.dot(Ainv, np.array([[v[i + 1]], [w[i + 1]]]))

    # Desired Motor Voltages
    Vdc[:, [i + 1]] = np.multiply(np.dot(GR, Ka), phi[:, [i + 1]])

    # Limit applied Vdc to within battery parameters
    if Vdc[0, i + 1] > Vbatt[i]:
        Vdc[0, i + 1] = Vbatt[i]
    else:
        if Vdc[0, i + 1] < - Vbatt[i]:
            Vdc[0, i + 1] = - Vbatt[i]
        if Vdc[1, i + 1] > Vbatt[i]:
            Vdc[1, i + 1] = Vbatt[i]
        else:
            if Vdc[1, i + 1] < - Vbatt[i]:
                Vdc[1, i + 1] = - Vbatt[i]

        # Calculate actual motor shaft velocities and
        # armature currents
    Omega[:, [i + 1]] = Omega[:, [i]] + np.dot(dt, (np.dot(B1, Omega[:, [i]]) + np.multiply(Kt, Ia[:, [i]]))) / Jm
    Ia[:, [i + 1]] = Ia[:, [i]] + np.dot(np.dot(dt, (1 / La)), (
                np.multiply(- Ka, Omega[:, [i]]) - np.multiply(Ra, Ia[:, [i]]) + Vdc[:, [i + 1]]))
    # Limit shaft velocities to no load speed if necessary:
    if Omega[0, i + 1] > 1200:
        Omega[0, i + 1] = 1200
    if Omega[1, i + 1] > 1200:
        Omega[1, i + 1] = 1200

    # battery model: four alkaline batteries in series
    Iabs[:, [i + 1]] = abs(Ia[:, [i + 1]])

    # energy equation to compute remaining energy in battery
    AH[i + 1] = AH[i] - (np.dot(dt, (Iabs[0, i] + np.dot(0.5, (Iabs[0, i + 1]) - Iabs[0, i])))) / 3600 - np.dot(dt, (
                Iabs[0, i] + np.dot(0.5, (Iabs[0, i + 1] - Iabs[0, i])))) / 3600

    # Battery model umath.sing a curve from panasonic
    Vbatt[i + 1] = Vbatt[0] - np.dot(0.72, (AH[0] - AH[i + 1])) - np.dot(Rbatt, (Iabs[0, i] + Iabs[1, i]))

    # Calculate the new position vector in global
    # frame, with GR correct:
    Ei[:, [i + 1]] = Ei[:, [i]] + np.dot(np.dot(np.dot(dt, np.array(
        [[math.cos(Ei[2, i]), - math.sin(Ei[2, i]), 0], [math.sin(Ei[2, i]), math.cos(Ei[2, i]), 0], [0, 0, 1]])),
                                                Jinv), np.array(
        [[np.dot(r1, Omega[0, i + 1]) / GR], [np.dot(r2, Omega[1, i + 1]) / GR], [0]]))
    # Plot new robot pose
    if i == 0:
        # plot the pose of the robot for the first timestep
        P = np.array([Ei[0, i], Ei[1, i]])
        O = Ei[2, i]
        c1 = P + np.array(
            [np.dot(x, math.cos(O)) + np.dot(l, math.sin(O)), np.dot(x, math.sin(O)) - np.dot(l, math.cos(O))])
        c2 = P + np.array(
            [np.dot(x, math.cos(O)) - np.dot(l, math.sin(O)), np.dot(x, math.sin(O)) + np.dot(l, math.cos(O))])
        c3 = P + np.array(
            [np.dot(- x, math.cos(O)) - np.dot(l, math.sin(O)), np.dot(- x, math.sin(O)) + np.dot(l, math.cos(O))])
        c4 = P + np.array(
            [np.dot(- x, math.cos(O)) + np.dot(l, math.sin(O)), np.dot(- x, math.sin(O)) - np.dot(l, math.cos(O))])
        M = np.array([[c1[0], c1[1]], [c2[0], c2[1]], [c3[0], c3[1]], [c4[0], c4[1]]])
    if np.mod(i, 2000) == 0:
        # plot the pose of the robot every 2000 timesteps
        P = np.array([Ei[0, i], Ei[1, i]])
        O = Ei[2, i]
        c1 = P + np.array(
            [np.dot(x, math.cos(O)) + np.dot(l, np.sin(O)), np.dot(x, math.sin(O)) - np.dot(l, math.cos(O))])
        c2 = P + np.array(
            [np.dot(x, math.cos(O)) - np.dot(l, math.sin(O)), np.dot(x, math.sin(O)) + np.dot(l, math.cos(O))])
        c3 = P + np.array(
            [np.dot(- x, math.cos(O)) - np.dot(l, math.sin(O)), np.dot(- x, math.sin(O)) + np.dot(l, math.cos(O))])
        c4 = P + np.array(
            [np.dot(- x, math.cos(O)) + np.dot(l, math.sin(O)), np.dot(- x, math.sin(O)) - np.dot(l, math.cos(O))])
        N = np.array([[c1[0], c1[1]], [c2[0], c2[1]], [c3[0], c3[1]], [c4[0], c4[1]]])
        M = np.concatenate((M, N), axis=0)
    # Increment iteration counter
    i = i + 1

# Plot end of run information
# X-Y plot of center of robot motion in global frame
plt.figure()
plt.title('X-Y Plot of Robot Position in Global Frame')
plt.plot(Ei[0, :], Ei[1, :])
plt.show()
plt.title('Motor 1 and 2 - Applied Voltages')
plt.plot(t, Vdc[0, :], t, Vdc[1, :])
plt.show()
plt.title('Motor 1&2 Currents')
plt.plot(t, Ia[0, :], t, Ia[1, :])
plt.show()
plt.title('Robot Path in Global Frame')
plt.plot(M[:, 0], M[:, 1])
plt.show()
plt.title('Battery voltage')
plt.plot(Vbatt)
plt.show()
# Wheel 1&2 Angular Velocity
plt.title('Wheel 1&2 Vel. (RPM)')
plt.plot(t, np.dot(30, Omega[0, :]) / (np.dot(np.pi, GR)), t, np.dot(30, Omega[1, :] / (np.dot(np.pi, GR))))
plt.show()
# Wheel 1&2 Linear Velocity
plt.title('Linear (Xr-direction) Vel. (m/sec)')
Vxr = (np.dot(r1, Omega[0, :]) + np.dot(r2, Omega[1, :])) / (np.dot(2, GR))
plt.plot(t, Vxr)
plt.show()
# X error and Y error
plt.title('X and Y errors in Global Frame (meters)')
plt.plot(t, e[0, :], e[1, :])
plt.show()
