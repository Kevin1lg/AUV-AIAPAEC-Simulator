import numpy as np
from scipy.linalg import eig
from lqg_control.a_ctrl_obs_eigen import Controllability, printObservability
from lib.main_loop import Espace
from lib.design_lineal_constans import constantes_ABCD

def check_ctr_obs(A,B,C,D):
    # Get the number of rows
    num_filas = len(A)

    # Get the number of columns (length of a row)
    num_columns = len(A[0])

    print('Matrix A:')
    print("Number of rows:", num_filas)
    print("Number of columns:", num_columns)

    print('Controllability and Observability of Matrices A, B and C')
    rcm=Controllability(A, B, C, D)
    rom=printObservability(A, B, C, D)

    # Compare if rcm is equal to d_A
    if np.array_equal(rcm, num_filas):
        print('The model is Controllable')
    else:
        print('The model is not controllable')

    # Compare if rom is equal to d_A
    if np.array_equal(rom, num_filas):
        print('The model is Observable')
    else:
        print('The model is not Observable')
    
    Espace()
    print('Additional Data:')
    print('If there are zero rows and/or columns in matrices A, B, C, it can affect whether the model is controllable and observable, and if it becomes controllable and observable, it can cause many errors in the control model.')
    
    def check_fc_zeros(A):
        # Check if any row is zero
        filas_cero = np.all(A == 0, axis=1)

        # Check if any column is zero
        columnas_cero = np.all(A == 0, axis=0)

        # Print results
        print("Rows with all zero elements:", np.where(filas_cero)[0])
        print("Columns with all zero elements:", np.where(columnas_cero)[0])
    print(' ')
    print('Matrix A:')
    check_fc_zeros(A)
    print(' ')   
    print('Matrix B:')
    check_fc_zeros(B)

