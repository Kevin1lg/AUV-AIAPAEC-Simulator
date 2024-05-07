# 4
import numpy as np

#***************************************************************************
def scaling_matrix():
    su = np.array([1/1, 1/1, 1/1] )
    sx = np.array( [1/2, 1/0.1, 1/(10*np.pi/180)] )
    sy = np.array( [1/2, 1/0.1, 1/(10*np.pi/180)] )

    # Scaling Matrices
    # unew = su uold
    # xnew = sx xold
    # ynew = sy yold
    su_diag=np.diag(su)
    sx_diag=np.diag(sx)
    sy_diag=np.diag(sy)
    return su_diag, sx_diag, sy_diag

def print_scaling_matrix(su_diag, sx_diag, sy_diag):
    print('\nMatrix  su')
    print(su_diag)
    print('\nMatrix  sx')
    print(sx_diag)
    print('\nMatrix  sy')
    print(sy_diag)

#***************************************************************************
# Scaled Dynamics
def scaled_dinamics(ap0,bp0,cp0,dp0,su_diag, sx_diag, sy_diag):
    ap=sx_diag @ ap0 @ np.linalg.inv(sx_diag)
    bp=sx_diag @ bp0 @ np.linalg.inv(su_diag)
    cp=sy_diag @ cp0 @ np.linalg.inv(sx_diag)
    dp=sy_diag @ dp0 @ np.linalg.inv(su_diag)
    return ap,bp,cp,dp

def print_scaled_dinamics(ap,bp,cp,dp):
    print('\nMatrix  ap')
    print(ap)
    print('\nMatrix  bp')
    print(bp)
    print('\nMatrix  cp')
    print(cp)
    print('\nMatrix  dp')
    print(dp)

