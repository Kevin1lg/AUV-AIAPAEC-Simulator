import numpy as np

def radio(x, a, b, c, aoff, theta, d, l, lf):
    """
    Hydrodynamic hull design
    """
    n = 2  # Exponential coefficient

    if x < a:
        R = 1/2 * d * (1 - ((x + aoff - a) / a)**2)**(1/n)  # Nose of the hoof
    elif x < (a + b):
        R = d / 2  # Middle of the hoof
    else:
        R = (1/2 * d - (3 * d / 2 / c**2 - np.tan(theta) / c) * (x - lf)**2 + 
             (d / c**3 - np.tan(theta) / c**2) * (x - lf)**3)  # Tail of the hoof

    return R
