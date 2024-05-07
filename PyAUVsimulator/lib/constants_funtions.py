import numpy as np

# Non-linear model
def inp_c():
    inp=np.array([0,0,0,0,0,0])
    return inp

def x0_c():
    x0 = [0,0,0,0,0,0]
    return x0

def Ni_c():
    Ni = 300
    t=np.linspace(0,50, Ni)
    return Ni,t
##***************************************************************************
# Initial constants U and X to linearize the nonlinear system
def constants_LINEARIZATION():
    t = 0
    U0=[10,2,0.1]
    X0=[1,0.1,2*np.pi/180,0.001,0.001,2*np.pi/180]
    return t, X0, U0

#***************************************************************************
# Define the angular frequency on a logarithmic scale
def angular_freq():
    w = np.logspace(-2, 3, 100)
    return w

def triv_sigma(g, w):
    m, p, _ = g.frequency_response(w)
    sjw = (m*np.exp(1j*p)).transpose(2, 0, 1)
    sv = np.linalg.svd(sjw, compute_uv=False)
    return sv