import pylab as plt
import numpy as np
from math import *


def ddstep(xc, yc, qc, r, l, dt, w1, w2):
    xn = xc + (r * dt / 2.0) * (w1 + w2) * cos(qc)
    yn = yc + (r * dt / 2.0) * (w1 + w2) * sin(qc)
    qn = qc + (r * dt / (2.0 * l)) * (w1 - w2)
    return (xn, yn, qn)


N = 200
x = np.zeros(N)
y = np.zeros(N)
q = np.zeros(N)
x[0] = 1;
y[0] = 2;
q[0] = 0.7
t = 0;
dt = 0.1
for i in range(N - 1):
    w1 = 0.1
    w2 = 0.5
    x[i + 1], y[i + 1], q[i + 1] = ddstep(x[i], y[i], q[i], 1.0, 6.0, dt, w1, w2)
    t = t + dt

plt.plot(x, y, 'b')
plt.show()
