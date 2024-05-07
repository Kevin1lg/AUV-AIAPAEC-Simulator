import numpy as np
from scipy.integrate import odeint
from lib.constants import constants_initial
from lib.constants_funtions import *

Gb, Inertia, datos_ext, dimensions, Parameters_water, dot, Xx, Yy, Nn= constants_initial()
inp=inp_c()
x0 = x0_c()
Ni,t=Ni_c()

def model_auv(inp,t, Gb, Inertia, datos_ext, dimensions, Parameters_water, dot, Xx, Yy, Nn, distance_propellers,Tp1, Tp2, Tp3):

    # Separation of variables
    xg, yg, zg, xb, yb, zb = Gb
    ix, iy, iz = Inertia
    m, wei, buo= datos_ext
    a, b, c, l, d = dimensions
    rho, N, N_kpdot, aoff, theta, lf = Parameters_water
    nrdot, xudot, yvdot, yrdot, nvdot = dot
    xrr, xvr, xuu = Xx
    yrr, yur, yuv, yvv = Yy
    nrr, nur, nuv, nvv = Nn
    dp1, dp2, dp3 = distance_propellers

    # Outputs
    u=inp[0]
    v=inp[1]
    r=inp[2]
    xpos=inp[3]
    ypos=inp[4]
    apsi=inp[5]

    # Constants
    c3=np.cos(apsi)
    s3=np.sin(apsi)

    # Mass matrix
    r2=(1/2)*rho*l**(2)
    r3=(1/2)*rho*l**(3)
    r4=(1/2)*rho*l**(4)
    r5=(1/2)*rho*l**(5)  

    MASS=np.array([ [m-r3*xudot,        0,                  -m*yg],
                    [0,           m-r3*yvdot,           m*xg-r4*yrdot],
                    [-m*yg,      m*xg-r4*nvdot,         iz-r5*nrdot]])   #3*3
    

    Minv=np.linalg.inv(MASS)

    g=np.array([[ 0],
                [ 0],
                [ 0 ]]) #1x6
    
    # Moments (K,M,N) Forces(X,Y,Z)
    X=m*(v*r+xg*(r**2))+r4*(+xrr*r**(2))+r3*(xvr*v*r)+r2*xuu*abs(u)*u+Tp1+Tp2+g[0]	         
    Y=m*(yg*(r**2))+r4*(yrr*r*abs(r))+r3*(yur*u*r)+r2*(yuv*u*v+yvv*v*abs(v))+Tp3+g[1]
    N=-m*(xg*(u*r)-yg*(-v*r))+r5*(nrr*r*abs(r))+r4*(nur*u*r)+r3*(nuv*u*v+nvv*v*abs(v))-(Tp1*dp1/2)+(Tp2*dp2/2)+(Tp3*dp3)+g[2]

    xdot=np.array([ [Minv[0,0]*X+Minv[0,1]*Y+Minv[0,2]*N],
                    [Minv[1,0]*X+Minv[1,1]*Y+Minv[1,2]*N],
                    [Minv[2,0]*X+Minv[2,1]*Y+Minv[2,2]*N],
                    [np.array([c3*u+s3*v])],
                    [np.array([s3*u+c3*v])],
                    [np.array([1*r])]])    #1x6

    ydot=[  xdot[0][0][0],
            xdot[1][0][0],
            xdot[2][0][0],
            xdot[3][0][0],
            xdot[4][0][0],
            xdot[5][0][0]]

    return ydot

