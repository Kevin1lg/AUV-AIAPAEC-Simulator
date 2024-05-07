# 1_
import numpy as np
from scipy.linalg import eig

#***************************************************************************
# Controllability 
def Controllability(ap0,bp0,cp0,dp0):
    print("\nControllability")
    cm1 = bp0
    cm2 = ap0 @ bp0
    cm3 = ap0 @ ap0 @ bp0

    # Create the Controllability Matrix by concatenating the matrices
    cm = np.column_stack((cm1, cm2, cm3))
    # Calculate the range of the Controllability Matrix
    rcm = np.linalg.matrix_rank(cm)
    #Print
    print("\nMatrix cm:")
    print(cm)
    print("\nRank of controllability matrix:", rcm)
    print('---------------------------------------------------------------------------------------')
    print('***************************************************************************************')
    return rcm

#***************************************************************************
# Observability
def printObservability(ap0,bp0,cp0,dp0):
    print("\nObservability")
    om1 = cp0
    om2 = cp0 @ ap0
    om3 = cp0 @ ap0 @ ap0

    # Create the Observability Matrix by concatenating the matrices
    om = np.vstack((om1, om2, om3))
    # Calculate the rank of the Observability Matrix
    rom = np.linalg.matrix_rank(om)
    
    print("\nMatrix om:")
    print(om)
    print("\nRank of observability matrix:", rom)
    print('---------------------------------------------------------------------------------------')
    print('***************************************************************************************')
    return rom

def note_ctr_obs(rcm,rom,ap0):
    # Get the number of rows
    num_filas = len(ap0)

    # Get the number of columns (length of a row)
    num_columns = len(ap0[0])
    # Compare if rcm is equal to d_A
    if np.array_equal(rcm, num_filas):
        print('The model is controllable')
    else:
        print('The model is not controllable')

    # Compare if rom is equal to d_A
    if np.array_equal(rom, num_filas):
        print('The model is observable')
    else:
        print('The model is not Observable')
    
#***************************************************************************
# Natural Modes: Poles (Eigenvalues), Eigenvectors
# evec contains eigenvectors
# eval contains poles or eigenvalues
def EingV(ap0):
    eval, evec = eig(ap0)
    return eval, evec

# Print eigenvalues ​​and eigenvectors
def printEingV( eval, evec):
    print("\nEigenvalues:")
    print(np.diag(eval))
    print("\nEigenvectors:")
    print(evec)
    print('---------------------------------------------------------------------------------------')
    print('***************************************************************************************')
    