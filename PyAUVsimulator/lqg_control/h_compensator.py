# 9_
import os
import numpy as np
import matplotlib.pyplot as plt
import control
from lib.constants_funtions import triv_sigma

    #***************************************************************************
    # Recover Target Loop By Solving Cheap LQR Problem
def LQR(a,b,c,nc):
    q=c.T @ c   # State Weighting Matrix
    rho=1e-03   # Cheap control recovery parameter;
    r=rho*np.eye(nc)    # Control Weigthing Matrix
    k, poles, g = control.care(a, b, 50*q, r)  # Compute Control Gain Matrix G
    return g
#***************************************************************************
# Compensator Analysis
def Compensator_analysis(a,b,c,g,h,nc,ns):
    ak=np.block([
        [a - b @ g - h @ c, np.zeros((ns+nc,nc))],
        [-g, np.zeros((nc,nc))]
    ])
    bk=np.block([
        [-h],
        [np.zeros((nc,nc))]
    ])
    ck=np.block([
        [np.zeros((nc,ns+nc)), np.eye((nc))]])
    dk=np.zeros((3,3))
    return ak,bk,ck,dk

def print_controlPA(ak,bk,ck,dk):
    print('\nMatrix  ak')
    print(ak)
    print('\nMatrix  bk')
    print(bk)
    print('\nMatrix  ck')
    print(ck)
    print('\nMatrix  dk')
    print(dk)

def compensador_SV(a,h,g,nc,ak,bk,ck,w,output_folder):
    sys7=control.ss(a, h, g, np.zeros((nc,nc))) # Compensator Zeros
    
    # plt.figure()
    print("\nClose the image to continue the simulation")
    cpoles, czeros = control.pzmap(sys7) # Check Compensator Zeros
    output_path = os.path.join(output_folder, '8_Check compensator zeros.png')
    plt.savefig(output_path)

    sys8=control.ss(ak, bk, ck, np.zeros((nc,nc)))
    plt.figure()
    polecheck, zerocheck = control.pzmap(sys8) #Compensator Zeros
    output_path = os.path.join(output_folder, '9_Compensator zeros.png')
    plt.savefig(output_path)

    sv8 = triv_sigma(sys8, w)
    plt.figure()
    plt.semilogx(w, 20*np.log10(sv8), label=r'$\sigma_1(S_1)$')
    plt.grid(True, which='both', linestyle='--', linewidth=0.5)
    plt.ylabel('Singular Values [dB]')
    plt.xlim([1e-2, 1e3])
    plt.xlabel('Frequency [rad/s]')
    plt.title('Compensator of augmented plant')
    output_path = os.path.join(output_folder, '10_Compensator of augmented plant.png')
    plt.savefig(output_path)
    plt.show()