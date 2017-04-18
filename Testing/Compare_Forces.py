import sys
import numpy as np

sys.path.append('..')
from Model.Forces import Forces
from Trim.Trim import forces_and_moments
from Trim.Trim import calculate_forces_moments

F = Forces()

Pn0 = 0.0
Pe0 = 0.0
Pd0 = 0.0
u0 = 1.99775573e+00
v0 = -8.78083636e-03
w0 = 9.43129556e-02
phi0 = 1.03206350e-03
theta0 = 1.56561088e+00
psi0 = 0.0
p0 = -1.33331541e-01
q0 = 7.13558498e-07
r0 = 6.91389867e-04
states0 = np.array([Pn0, Pe0, Pd0, u0, v0, w0, phi0, theta0, psi0, p0, q0, r0])

Va = 15.0
alpha = 0.125
beta = 0.221

deltaA0 = -0.05254372966228464
deltaE0 = -0.0994183623225
deltaR0 = -0.20465031234498585
deltaT0 = 0.401866747909
control_surfaces = np.array([deltaA0, deltaE0, deltaR0, deltaT0])

f1 = calculate_forces_moments(states0, Va, alpha, beta, control_surfaces)

f2 = F.getForces(states0, Va, alpha, beta, control_surfaces)

diff = f1-f2

print(diff)