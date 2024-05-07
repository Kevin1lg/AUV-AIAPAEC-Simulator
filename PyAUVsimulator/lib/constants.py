import numpy as np
def constants_initial():
    #**********---------------------INITIAL CONSTANTS---------------------*********
    # Center of gravity
    xg=0.10
    yg=0
    zg=0.03

    # Center of buoyancy
    xb=0.10
    yb=0.10
    zb=-0.10
    
    # Inertia moments
    ix=1.1866349780349186   # Inertia moments about the x axis
    iy=8.449738344749418    # Inertia moments about the y axis
    iz=8.449738344749418    # Inertia moments about the z axis
    

    m=30 #kg
    wei=m*9.8
    buo=wei
    
    #AIAPAEC geometry characteristics
    a=2.000e-1   # length of the nose 
    b=8.100e-1   # length of the middle 
    c=3.000e-1   # length of the tail
    l=1.2500     # total length
    d=1.8000e-1    # maximum diameter of the hull
    
    rho=1.030e+3   # ocean water density
    N=1000         # number of iterations for integration
    N_kpdot=1000   # number of iterations for kpdot integration
    aoff=0         # length from the cut nose to the edge Myring function
    theta=4.36e-1  # angle of the Tail
    lf=a+b-aoff
    

    nrdot=-0.0044     
    xudot=-7.7800e-04   
    yvdot=-0.0503       
    yrdot=0.0065        
    nvdot=0.0065        
    
    #X
    xrr=-0.0065 
    xvr=0.0503 
    xuu=-0.0018 
    
    #Y
    yrr=-7.5500e-04 
    yur=0.0029 
    yuv=-0.0373 
    yvv= -0.1560 
    
    #N
    nrr=-0.0040 
    nur=0.0054 
    nuv=-4.14e-02 
    nvv=-5.3500e-04 
    #**********---------------------INITIAL CONSTANTS---------------------*********

    #define matrix
    Gb = np.block([xg, yg, zg, xb, yb, zb])
    Inertia = np.block([ix, iy, iz])
    datos_ext=np.block([m, wei, buo])
    dimensions = np.block([a, b, c, l, d])
    Parameters_water=np.block([rho, N, N_kpdot, aoff, theta, lf])
    dot=np.block([nrdot, xudot, yvdot, yrdot, nvdot])
    Xx=np.block([xrr, xvr, xuu])
    Yy=np.block([yrr, yur, yuv, yvv])
    Nn=np.block([nrr, nur, nuv, nvv])

    return Gb, Inertia, datos_ext, dimensions, Parameters_water, dot, Xx, Yy, Nn

def distance_propel():
    dp1=0.2500      # distance between thrusters P1 and P2
    dp2=0.2500 
    dp3=0.2650      # distance of P3 cpn to the center of gravity
    distance_propellers=np.block([dp1, dp2, dp3])
    return distance_propellers

def input_tp(number1,number2,number3):
    Tp1=number1
    Tp2=number2
    Tp3=number3
    return Tp1, Tp2, Tp3

    