import pylab as plt
import numpy as np
from math import *
from matplotlib.patches import Ellipse
from numpy.linalg import linalg


def wheels(t):
    if (t < 1.5):
        w1 = 1.0
        w2 = 1.0
        return w1, w2
    if (t < 3):
        w1 = 2.0
        w2 = 1.0
        return w1, w2
    w1 = 1.0
    w2 = 1.0
    return w1, w2


r = 20.0
l = 12.0
N = 10
dt = 0.01
Tend = 5
NumP = int(Tend / dt)

mu1, sigma1 = 0.0, 0.05
mu2, sigma2 = 0.0, 0.01
tp = np.arange(0, Tend, dt)

xpath = np.zeros((N, NumP))
ypath = np.zeros((N, NumP))
thpath = np.zeros((N, NumP))

for k in range(N):
    thv = 0.0
    xv = 0.0
    yv = 0.0
    i = 0
    xerr = np.random.normal(mu1, sigma1, NumP)
    yerr = np.random.normal(mu1, sigma1, NumP)
    therr = np.random.normal(mu2, sigma2, NumP)
    for t in tp:
        w1, w2 = wheels(t)
        dx = (r * dt / 2.0) * (w1 + w2) * cos(thv) + xerr[i]
        dy = (r * dt / 2.0) * (w1 + w2) * sin(thv) + yerr[i]
        dth = (r * dt / (2.0 * l)) * (w1 - w2) + therr[i]
        xv = xv + dx
        yv = yv + dy
        thv = thv + dth
        xpath[k][i] = xv
        ypath[k][i] = yv
        thpath[k][i] = thv
        i = i + 1
[x, y] = [xpath[:,(xpath.shape[1]) - 1], ypath[:,(ypath.shape[1]) -1 ]]

for k in range(N):
    plt.plot(xpath[k], ypath[k], 'b-')
plt.xlim(-3, 130)
plt.ylim(-20, 130)
plt.show()

plt.plot(xpath[-1], ypath[-1], 'b-', x, y, 'r.')
plt.xlim(-3, 130)
plt.ylim(-20, 130)
plt.show()

s = 2.447651936039926
mat = np.array([x, y])
cmat = np.cov(mat)
evals, evec = linalg.eigh(cmat)
r1 = 2 * s * sqrt(evals[0])
r2 = 2 * s * sqrt(evals[1])
angle = 180 * atan2(evec[0, 1], evec[0, 0]) / np.pi
ell = Ellipse((np.mean(x), np.mean(y)), r1, r2, angle)
a = plt.subplot(111, aspect='equal')
ell.set_alpha(0.2)
a.add_artist(ell)

plt.plot(xpath[-1], ypath[-1], 'b-', x, y, 'r.')
plt.xlim(-3, 130)
plt.ylim(-20, 130)
plt.show()
