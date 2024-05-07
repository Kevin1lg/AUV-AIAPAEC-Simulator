import numpy as np
from lib.constants import *

Gb, Inertia, datos_ext, dimensions, Parameters_water, dot, Xx, Yy, Nn= constants_initial()

def model_auv_lin(t, X, U,Gb, Inertia, datos_ext, dimensions, Parameters_water, dot, Xx, Yy, Nn, distance_propellers, **params):  
    aa = params['aa']

    # Separation of variables
    xg, yg, zg, xb, yb, zb = Gb
    ix, iy, iz = Inertia
    m, wei, buo = datos_ext
    a, b, c, l, d = dimensions
    rho, N, N_kpdot, aoff, theta, lf = Parameters_water
    nrdot, xudot, yvdot, yrdot, nvdot = dot
    xrr, xvr, xuu = Xx
    yrr, yur, yuv, yvv = Yy
    nrr, nur, nuv, nvv = Nn
    dp1, dp2, dp3 = distance_propellers
    
    # Outputs
    u=X[0]
    v=X[1]
    r=X[2]
    xpos=X[3]
    ypos=X[4]
    apsi=X[5]

    Tp1=U[0]
    Tp2=U[1]
    Tp3=U[2]

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
    X=m*(v*r+xg*(r**2))+r4*(+xrr*r**(2))+r3*(xvr*v*r)+r2*xuu*abs(u)*u+Tp1+Tp2+g[0]	         #% propeller force and drag
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

    # LINEALIZATION    
    f1 = ydot[0]
    f2 = ydot[1]
    f3 = ydot[2]
    f4 = ydot[3]
    f5 = ydot[4]
    f6 = ydot[5]
    
    return np.array([f1, f2, f3, f4, f5, f6], dtype = np.float64)


def dfA(t, x, u,Gb, Inertia, datos_ext, dimensions, Parameters_water, dot, Xx, Yy, Nn, distance_propellers, **params):

    eps = 1e-10
    J = np.zeros([len(x), len(x)], dtype = np.float64)

    for i in range(len(x)):
        xl = x.copy() 
        xr = x.copy()
        ul = u.copy()
        ur = u.copy()

        xl[i] -= eps # finite differences respect to x
        xr[i] += eps

        fl = model_auv_lin(t, xl, ul,Gb, Inertia, datos_ext, dimensions, Parameters_water, dot, Xx, Yy, Nn, distance_propellers, **params)
        fr = model_auv_lin(t, xr, ur,Gb, Inertia, datos_ext, dimensions, Parameters_water, dot, Xx, Yy, Nn, distance_propellers, **params)

        J[ : , i] = (fr - fl) / (2 * eps) # central diff

    return J

def dfB(t, x, u,Gb, Inertia, datos_ext, dimensions, Parameters_water, dot, Xx, Yy, Nn, distance_propellers, **params):

    eps = 1e-10
    J = np.zeros([len(x), len(u)], dtype = np.float64)

    for i in range(len(u)):
        xl = x.copy()
        xr = x.copy()
        ul = u.copy()
        ur = u.copy()

        ul[i] -= eps # finite differences respect to u
        ur[i] += eps

        fl = model_auv_lin(t, xl, ul,Gb, Inertia, datos_ext, dimensions, Parameters_water, dot, Xx, Yy, Nn, distance_propellers, **params)
        fr = model_auv_lin(t, xr, ur,Gb, Inertia, datos_ext, dimensions, Parameters_water, dot, Xx, Yy, Nn, distance_propellers, **params)

        J[ : , i] = (fr - fl) / (2 * eps)

    return J