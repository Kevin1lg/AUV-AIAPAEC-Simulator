import numpy as np
from scipy.integrate import trapz
from constants import *  
from radio import radio  
from createfigure import createfigure, createfigure2
# Create a vector x with N evenly spaced values from 0 to l
x = np.linspace(0, l, N)

# Initialize a vector r with N zeros
r = np.zeros(N)

# Calculate the vector r using the radio function
for i in range(N):
    xi = x[i]
    r[i] = radio(xi, a, b, c, aoff, theta, d, l, lf)

# Create a figure using the createfigure function
createfigure(x, r, d, l)

# Initialize a vector R1 with N zeros
R1 = np.zeros(N)
# Reverse the order of the elements in r and store it in R1
for i in range(1, N+1):
    R1[i-1] = r[N-i]

# Create a new vector x with N evenly spaced values from xt to xb2
x = np.linspace(xt, xb2, N)
# Create a new figure using the createfigure2 function
createfigure2(x, R1, d, l)

# Calculate ma and maf

ma = np.pi * rho * R1**2
maf = np.pi * rho * (aflap**2 - R1**2 + R1**4 / aflap**2)

# Calculate f1, f2, f3, f4
f1 = x * ma
f2 = x**2 * ma
f3 = x * maf
f4 = x**2 * maf

# Integration to get added mass m22
int_m22 = 0
for i in range(N-1):  
    if x[i] < xf:
        int_m22 += 0.5 * (ma[i] + ma[i+1]) * (x[i+1] - x[i])
    elif x[i] < xf2:
        int_m22 += 0.5 * (maf[i] + maf[i+1]) * (x[i+1] - x[i])
    else:
        int_m22 += 0.5 * (ma[i] + ma[i+1]) * (x[i+1] - x[i])

m22 = int_m22

# Alternative method using numpy trapz function
fm22 = np.zeros(N)
for i in range(N):
    if x[i] < xf:
        fm22[i] = ma[i]
    elif x[i] < xf2:
        fm22[i] = maf[i]
    else:
        fm22[i] = ma[i]
m22_trapz = trapz(fm22, x)

# Integration to get added mass m53
int_m53 = 0
for i in range(N-1):
    if x[i] < xf:
        int_m53 += 0.5 * (f1[i] + f1[i+1]) * (x[i+1] - x[i])
    elif x[i] < xf2:
        int_m53 += 0.5 * (f3[i] + f3[i+1]) * (x[i+1] - x[i])
    else:
        int_m53 += 0.5 * (f1[i] + f1[i+1]) * (x[i+1] - x[i])
m53 = -int_m53

# Alternative method using numpy trapz function
fm53 = np.zeros(N)
for i in range(N):
    if x[i] < xf:
        fm53[i] = f1[i]
    elif x[i] < xf2:
        fm53[i] = f3[i]
    else:
        fm53[i] = f1[i]
m53_trapz = -trapz(fm53, x)

# Integration to get added mass m55
int_m55 = 0
for i in range(N-1):
    if x[i] < xf:
        int_m55 += 0.5 * (f2[i] + f2[i+1]) * (x[i+1] - x[i])
    elif x[i] < xf2:
        int_m55 += 0.5 * (f4[i] + f4[i+1]) * (x[i+1] - x[i])
    else:
        int_m55 += 0.5 * (f2[i] + f2[i+1]) * (x[i+1] - x[i])
m55 = int_m55

# Alternative method using numpy trapz function
fm55 = np.zeros(N)
for i in range(N):
    if x[i] < xf:
        fm55[i] = f2[i]
    elif x[i] < xf2:
        fm55[i] = f4[i]
    else:
        fm55[i] = f2[i]
m55_trapz = trapz(fm55, x)

# Added mass in the SNAME standard form
yvdot = -m22
zwdot = yvdot
mwdot = -m53
nvdot = -mwdot
yrdot = nvdot
zqdot = mwdot
mqdot = -m55
nrdot = mqdot

# Rolling added mass according to Blevins formula m44
N_kpdot = 1000
int_kpdot = 0
a_mean = aflap  # average height of the flap wrt vehicle centerline
f5 = (2 / np.pi) * rho * a_mean**4 * np.ones(N_kpdot)
Xflap = np.linspace(xf, xf2, N_kpdot)  # define the Xflap vector
for i in range(N_kpdot - 1):
    int_kpdot += 0.5 * (f5[i] + f5[i+1]) * (Xflap[i+1] - Xflap[i])
kpdot = -int_kpdot

# Axial added mass -- Blevins, pg. 407
m11 = (4 * np.pi * rho * alph / 3) * (l / 2) * (d / 2)**2
xudot = -m11

# Crossflow added mass
xwq = zwdot
yur = xudot
zuq = -xudot
muwa = -(zwdot - xudot)
nuva = -(xudot - yvdot)
xqq = zqdot
ywp = -zwdot
zvp = yvdot
mvp = -yrdot
nwp = zqdot
xvr = -yvdot
ypq = -zqdot
zrp = yrdot
mrp = (kpdot - nrdot)
npq = -(kpdot - mqdot)
xrr = -yrdot
muq = -zqdot
nur = yrdot
