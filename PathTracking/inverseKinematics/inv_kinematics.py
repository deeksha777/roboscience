import pylab as plt
import numpy as np
from math import *

N=100
t0 = 0.0
t1 = 2.0
t = np.linspace(t0,t1,N)
dt = (t1-t0)/N
one = np.ones((N))
xp = np.zeros((N))
yp = np.zeros((N))
th = np.zeros((N))

x = t*t
y = t

plt.figure()
plt.plot(x,y,'g-')
plt.legend(['Path'],loc='best')
plt.title('Quadratic Path')
plt.show()

doty=one
dotx=2*t
ddoty=0
ddotx=2*one

r = 1.0
L = 4.0
v = np.sqrt(dotx*dotx + doty*doty)
kappa = (dotx*ddoty - doty*ddotx)/(v*v*v)
dotphi1 = (v/r)*(kappa*L +1)
dotphi2 = (v/r)*(-kappa*L+1)

plt.plot(t,dotphi1,'b-', t,dotphi2,'g-')
plt.title('Wheel Speeds')
plt.legend(['Right', 'Left'],loc='best')
plt.show()

xp[0] = 0.0
yp[0] = 0.0
th[0] = 1.5707963267949

for i in range(N-1):
    xp[i+1] = xp[i] + (r*dt/2.0) * (dotphi1[i]+dotphi2[i]) * cos(th[i])
    yp[i+1] = yp[i] + (r*dt/2.0)*(dotphi1[i]+dotphi2[i])* sin(th[i])
    th[i+1] = th[i] + (r*dt/(2.0*L))*(dotphi1[i]-dotphi2[i])

plt.figure()
plt.plot(x,y,'g-', xp, yp, 'bx')
plt.legend(['Original Path', 'Robot Path'],loc='best')
plt.title('Path')
plt.show()