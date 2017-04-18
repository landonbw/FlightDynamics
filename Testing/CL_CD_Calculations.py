import sys
from matplotlib import pyplot as plt
import numpy as np

sys.path.append('..')
from Model.Forces import Forces

f = Forces()
angDeg = np.linspace(-80, 80, 100).tolist()
angRad = np.radians(angDeg)

cl = []
cd = []
for ang in angRad:
    sig = f.Sigma_Alpha(ang)
    CLval = f.CL_Alpha(ang, sig)
    CDval = f.CD_Alpha(ang)
    cl.append(CLval)
    cd.append(CDval)

plt.figure(1)
plt.plot(angDeg, cl)
plt.ylim((-3,3))
plt.title('Lift Coeficient')

plt.figure(2)
plt.plot(angDeg, cd)
plt.title('Drag Coeffcient')
plt.xlim((-20,80))
plt.grid(True)
plt.show()