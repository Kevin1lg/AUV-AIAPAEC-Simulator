import numpy as np
#***************************************************************************
#---------- Design of the quadratic optimal regulatory system---------
# linearization constants
def constantes_ABCD(A,B):
    A=A
    B=B
    C=np.block([np.identity(3), np.zeros((3, 3))])
    D=np.zeros((3, 3))
    return A, B, C, D

#***************************************************************************
# Constants to carry out its control
def constantes_abcdp(A,B,C,D):
    ap0=np.block([
    [A[0:3, 0:3] ]
    ])
    bp0=np.block([
    [B[0:3, 0:3]]
    ])
    cp0=np.identity(3)
    dp0=np.zeros((3,3))

    return ap0, bp0, cp0, dp0

def printConstsLineal(A,B,C,D):
    print("Matrix A:")
    print(A)
    print("\nMatrix B:")
    print(B)
    print("\nMatrix C:")
    print(C)
    print("\nMatrix D:")
    print(D)

def printConstsControl(ap0,bp0,cp0,dp0):
    # Prints each matrix (ap, bp, cp, dp)
    print("\nMatrix ap0:")
    print(ap0)
    print("\nMatrix bp0:")
    print(bp0)
    print("\nMatrix cp0:")
    print(cp0)
    print("\nMatrix dp0:")
    print(dp0)
