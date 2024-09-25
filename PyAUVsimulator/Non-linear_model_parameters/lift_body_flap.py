import numpy as np
from scipy.integrate import trapz
from constants import *  

# Smith technique using by Hoerner
zuwl = -cyd_dot * (180 / np.pi) * (1 / 2 * rho * l * d)
yuvl = zuwl  # axis=symmetry

zuwl = (2 * l**2 / rho) * zuwl
yuvl = (2 * l**2 / rho) * yuvl

# Prestero using Hoerner and Triantafillou
print('The Lift coefficient for the body')
zuwl = -1 / 2 * rho * (d)**2 * cyd
yuvl = zuwl

# Output results
print(f"zuwl: {zuwl}, yuvl: {yuvl}")

# Body Lift moments
print('The moment coefficients for the body')
muwl = -abs(x_hlift) * abs(zuwl)  # Use hand-right rule for rotation signs
nuvl = -muwl

print(f"muwl: {muwl}, nuvl: {nuvl}")

# Flap lift (see Hoerner pg. 3-2)
# ARe=2*(AR);           # two flaps side to side wrt the vehicle
# afac=0.9;             # Hoerner value

print('The hydrodynamic derivative for the Flap')
Clalpha = (1 / 2 / afac / np.pi + 1 / np.pi / AR + 1 / 2 / np.pi / (AR)**2)**-1  # Upper flap only

print(f"Clalpha: {Clalpha}")

print('The Lift coefficients for the Flap')
yuudelr = (2 * 1 / 2) * rho * Clalpha * Sflap
yuvf = -yuudelr
zuudels = -(2 * 1 / 2) * rho * Clalpha * Sflap
zuwf = zuudels
yurf = -(2 * 1 / 2) * rho * Clalpha * Sflap * xflap
zuqf = -yurf

print(f"yuudelr: {yuudelr}, yuvf: {yuvf}")
print(f"zuudels: {zuudels}, zuwf: {zuwf}")
print(f"yurf: {yurf}, zuqf: {zuqf}")

print('The moment coefficients for the Flap')
# Flap moment coefficients
muudels = (2 * 1 / 2) * rho * Clalpha * Sflap * xflap
muwf = muudels
nuudelr = (2 * 1 / 2) * rho * Clalpha * Sflap * xflap  # nuudelr=yuudelr*xflap;
nuvf = -nuudelr
muqf = -(2 * 1 / 2) * rho * Clalpha * Sflap * xflap**2
nurf = muqf
print(f"muudels: {muudels}, muwf: {muwf}")
print(f"nuudelr: {nuudelr}, nuvf: {nuvf}")
print(f"muqf: {muqf}, nurf: {nurf}")