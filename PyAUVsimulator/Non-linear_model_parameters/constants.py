import numpy as np

# AIAPAEC geometry characteristics
a = 2.000e-1   # length of the nose
b = 8.100e-1   # length of the middle
c = 3.000e-1   # length of the tail
l = 1.2500     # total length
d = 1.8000e-1  # maximum diameter of the hull

rho = 1.030e+3  # ocean water density
N = 1000        # number of iterations for integration
N_kpdot = 1000  # number of iterations for kpdot integration
aoff = 0        # length from the cut nose to the edge Myring function
theta = 4.36e-1 # angle of the Tail
lf = a + b - aoff

aflap = 2.6495e-1   # Flap height wrt centre line of the hull
Af = np.pi * (d/2)**2  # vehicle frontal area
Ap = l * d            # the vehicle plan area

# Axial added mass
alph = 0.03585

# Additional data for integration limits
dc = 6.50e-1  # distance from nose to centre of Body Frame Reference
dxb = dc

dxg = dc - 0.010  # distance from nose to Centre of Gravity 10mm
xt = -(l - dc)    # Aft end of the Tail
xt2 = -(a + b + 0.12112 - dc)  # Forward end of the Tail
xf = xt2          # Aft end of Flap
xf2 = xf + 0.12112  # Forward end of Flap
xb = dc - a       # Aft end of Nose
xb2 = dc          # Forward end of the Nose

Sflap = (0.110 + 0.060)/2 * 0.180  # area of fin (only one flap)
xflap = xf2 - 0.01  # fin location wrt CB origin
x_hlift = -0.7*l + dc  # hull lift pressure location

bflap = (0.110 + 0.060) / 2  # (B + b) / 2

# Constants for DRAG
css = 3.397e-3   # Schoenherr's value for flat plate skin friction
cdc = 1.1        # Drag coefficient for a cylindrical body
tr = 6/9         # taper ratio
cdf = 0.1 + 0.7 * tr  # Crossflow drag formula by Whicker and Fehlner

# Constants for Lift
cyd_dot = 0.003  # for l/2 in [6.7,10]
cybeta = 0.003 * (180/np.pi)
cyd = (l/d) * cybeta
AR = (bflap * bflap / Sflap)  # aspect ratio
ARe = 2 * AR  # two flaps side to side wrt the vehicle
afac = 0.9  # Hoerner value in degrees

# Distance_propel
dp1 = 0.2500  # distance between thrusters P1 and P2
dp2 = 0.2500 
dp3 = 0.2650  # distance of P3 cpn to the center of gravity
dp4 = 0.2650

def input_tp(number1, number2, number3):
    Tp1 = number1
    Tp2 = number2
    Tp3 = number3
    return Tp1, Tp2, Tp3
