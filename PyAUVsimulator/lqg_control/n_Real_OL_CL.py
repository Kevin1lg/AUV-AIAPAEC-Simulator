# 16
import os
import numpy as np
from scipy.signal import lti, lsim
import matplotlib.pyplot as plt
import control
from control import step_response
from control.matlab import pzmap
from lib.constants_funtions import triv_sigma, angular_freq

w=angular_freq()
#Real open loop analysis

def olreal(ap0,bp0,cp0,akr,bkr,ckr,ns,nc):
    alr=np.block([
        [ap0, bp0 @ ckr],
        [np.zeros((ns+nc+nc,ns)),akr]
    ])
    blr=np.block([
        [np.zeros((ns,nc))],
        [bkr]
    ])
    clr=np.block([
        [cp0, np.zeros((nc,ns+nc+nc))]])
    dlr=np.zeros((3,3))
    return alr,blr,clr,dlr

def print_COL(alr,blr,clr,dlr):
    print('\nMatrix  alr')
    print(alr)
    print('\nMatrix  blr')
    print(blr)
    print('\nMatrix  clr')
    print(clr)
    print('\nMatrix  dlr')
    print(dlr)

def Zeros_RF_OL(alr,blr,clr,dlr,w,tsv,output_folder):
    sysr1=control.ss(alr, blr, clr, dlr)
    plt.figure()
    olpoles, olzeros= pzmap(sysr1) # Open Loop Zeros
    # output_path = os.path.join(output_folder, '32_Open Loop Zeros.png')
    # plt.savefig(output_path)
    print('\nolpoles')
    print(olpoles)
    print('\nolzeros')
    print(olzeros)

    svr1 = triv_sigma(sysr1, w)
    print("\nClose the image to continue the simulation")
    plt.figure()
    plt.semilogx(w, 20*np.log10(svr1), label=r'$\sigma_1(S_1)$')
    plt.semilogx(w, tsv, label=r'$\sigma_1(S_1)$')
    plt.grid(True, which='both', linestyle='--', linewidth=0.5)
    plt.xlim([1e-2, 1e3])
    plt.ylabel('Singular Values [dB]')
    plt.xlabel('Frequency [rad/s]')
    plt.title('Open loop target sensitivity (S) and target loop')
    output_path = os.path.join(output_folder, '21_Open loop target sensitivity (S) and target loop.png')
    plt.savefig(output_path)
    plt.show()

#***************************************************************************
# Closed Loop Analysis

def clreal_FR(alr,blr,clr,dlr,w,nc,output_folder):
    sysclr=control.ss(alr-blr @ clr, blr, -clr, np.eye((nc)))
    svclr = triv_sigma(sysclr, w)
    sysclr1=control.ss(alr-blr @ clr, blr, clr, np.zeros((nc,nc)))
    svclr1= triv_sigma(sysclr1, w)
    sysclr2=lti(alr-blr @ clr, blr, clr, np.zeros((nc,nc)))
    return sysclr,svclr,sysclr1,svclr1,sysclr2


def plot_FR_CL(sysclr,svclr,sysclr1,svclr1,w,output_folder):
    print("\nClose the image to continue the simulation")
    plt.figure()
    plt.subplot(2, 1, 1)  # 2 rows, 1 columns, first subgraph
    plt.semilogx(w, 20*np.log10(svclr), label=r'\\sigma_1(S_1)')
    plt.grid(True, which='both', linestyle='--', linewidth=0.5)
    plt.ylabel('Singular Values [dB]')
    plt.xlim([1e-2, 1e3])
    plt.xlabel('Frequency [rad/s]')
    #plt.legend()
    plt.title('Sensitivity (S)')

    plt.subplot(2, 1, 2)  # 2 rows, 1 columns, second subgraph
    plt.semilogx(w, 20*np.log10(svclr1), label=r'$\sigma_1(S_1)$')
    plt.grid(True, which='both', linestyle='--', linewidth=0.5)
    plt.xlim([1e-2, 1e3])
    plt.ylabel('Singular Values [dB]')
    plt.xlabel('Frequency [rad/s]')
    #plt.legend()
    plt.title('Complementary sensitivity (T)')

    plt.tight_layout()  # Adjusts subgraphs so that they do not overlap

    output_path = os.path.join(output_folder, '22_Sensitivity and complementary sensitivity of the real system.png')
    plt.savefig(output_path)
    plt.show()

#*************************************************************************** 
def SR_CL(sysclr1,output_folder):
    t, y = step_response(sysclr1)
    plt.figure(figsize=(12, 6))
    plt.subplot(2, 2, 1)  # 2 rows, 2 columns, first subgraph
    plt.plot(t,y[0,0,:],label='u [m/s] ')
    plt.plot(t,y[1,0,:],label='v [m/s] ')
    plt.plot(t,y[2,0,:],label='r [rad/s] ')
    plt.grid()
    plt.xlim([0, 15])
    plt.ylabel('Amplitude')
    plt.xlabel('Time [s]')
    plt.legend(['u [m/s] ', 'v [m/s] ', 'r [rad/s] '])
    plt.title('Step response in closed loop caused by input 1')
    
    #plt.show()

    plt.subplot(2, 2, 2)  # 2 rows, 2 columns, second subgraph
    plt.plot(t,y[0,1,:],label='u [m/s] ')
    plt.plot(t,y[1,1,:],label='v [m/s] ')
    plt.plot(t,y[2,1,:],label='r [rad/s] ')
    plt.grid()
    plt.xlim([0, 15])
    plt.ylabel('Amplitude')
    plt.xlabel('Time [s]')
    plt.legend(['u [m/s] ', 'v [m/s] ', 'r [rad/s] '])
    plt.title('Step response in closed loop caused by input 2')
    #plt.show()

    plt.subplot(2, 2, 3)  # 2 rows, 2 columns, 3rd subgraph
    plt.plot(t,y[0,2,:], label='u [m/s] ')
    plt.plot(t,y[1,2,:], label='v [m/s] ')
    plt.plot(t,y[2,2,:], label='r [rad/s] ')
    plt.grid()
    plt.xlim([0, 15])
    plt.ylabel('Amplitude')
    plt.xlabel('Time [s]')
    plt.legend(['u [m/s] ', 'v [m/s] ', 'r [rad/s] '])
    plt.title('Step response in closed loop caused by input 3')
    plt.tight_layout()  # Adjusts subgraphs so that they do not overlap

    output_path = os.path.join(output_folder, '23_Step response in closed loop of the Real System by input 1,2,3.png')
    plt.savefig(output_path)
    
# lsim

def LSIM_CL(sysclr2):
    
    t = np.arange(0, 15.1, 0.1)  # Create an array of time values
    x = np.zeros((12, 1)).flatten()  # Convert to a one-dimensional array
    
    # Create a matrix u with zeros of the same size as t
    u = np.zeros((len(t), 3))
    # Assign 1 to the first column of u
    u[:, 0] = 1
    u[:, 2] = 10*np.pi/180
    # Simulate system response
    t, y, _ = lsim(sysclr2, u, t, x)
    return t,y

def plot_LSIM_CL(t,y,output_folder):
    # Plot the answer
    plt.figure()
    plt.plot(t, y)
    plt.grid()
    plt.ylabel('Amplitude')
    plt.xlabel('Time [s]')
    plt.legend(['u [m/s]', 'v [m/s]', 'r [rad/s]'])
    plt.title('Response in closed loop by u=[1 0 10*pi/180]')
    
    output_path = os.path.join(output_folder, '24_Response in closed loop of the real system by u[1] r[10].png')
    plt.savefig(output_path)

    plt.show()
    
