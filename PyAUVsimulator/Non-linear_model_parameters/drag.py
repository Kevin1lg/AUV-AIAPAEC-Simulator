import numpy as np
from scipy.integrate import trapz
from constants import *  
from radio import radio 

# Triantafillou Drag Formula
cd = css * np.pi * Ap / Af * (1 + 60 * (d / l)**3 + 0.0025 * (l / d))
xuu = -1 / 2 * rho * cd * Af

# Define x vector and calculate r
x = np.linspace(0, l, N)
r = np.array([radio(xi, a, b, c, aoff, theta, d, l, lf) for xi in x])

# Coordinate translation and inversion of x to set origin at Buoyancy Centre
R = np.array([r[N - (i + 1)] for i in range(N)])
x = np.linspace(xt, xb2, N)  # Overwrite the x vector

# Alternative method using numpy trapz function
fd1 = 2 * R
fd2 = 2 * x * R
fd3 = 2 * x * np.abs(x) * R
fd4 = 2 * x**2 * np.abs(x) * R

# Integration
int_fd1 = trapz(fd1, x)
int_fd2 = trapz(fd2, x)
int_fd3 = trapz(fd3, x)
int_fd4 = trapz(fd4, x)

# Techniques based in Prestero (2001)
print('According to Prestero Equations -- there are possible mistakes')
yvv = -1 / 2 * rho * cdc * int_fd1 - 2 * (1 / 2 * rho * Sflap * cdf)
mww = -1 / 2 * rho * cdc * int_fd2 - 2 * xflap * (1 / 2 * rho * Sflap * cdf)
yrr = -1 / 2 * rho * cdc * int_fd3 - 2 * xflap * np.abs(xflap) * (1 / 2 * rho * Sflap * cdf)
mqq = -1 / 2 * rho * cdc * int_fd4 - 2 * xflap**3 * (1 / 2 * rho * Sflap * cdf)

print(f"Prestero: yvv={yvv}, mww={mww}, yrr={yrr}, mqq={mqq}")

# Techniques based in Rentschler (2001)
print('According to Rentschler and modified sign of second terms')
print('The coefficients are quite actual')
yvv = -1 / 2 * rho * (cdc * int_fd1 + 2 * Sflap * cdf)
mww = 1 / 2 * rho * (cdc * int_fd2 + 2 * xflap * Sflap * cdf)
yrr = -1 / 2 * rho * (cdc * int_fd3 + 2 * xflap * np.abs(xflap) * Sflap * cdf)
mqq = -1 / 2 * rho * (cdc * int_fd4 + 2 * xflap**2 * np.abs(xflap) * Sflap * cdf)

print(f"Rentschler: yvv={yvv}, mww={mww}, yrr={yrr}, mqq={mqq}")

# The other drag coefficients due to symmetry of the body
zww = yvv
nvv = -mww
zqq = -yrr
nrr = mqq
kpp = 0  # obtained in experimental test, the value could be neglected

print(f"Symmetry: zww={zww}, nvv={nvv}, zqq={zqq}, nrr={nrr}, kpp={kpp}")