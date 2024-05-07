# 15_ 
import os
import numpy as np
import matplotlib.pyplot as plt
import pickle
import control
from control import step_response
from lib.constants_funtions import triv_sigma, angular_freq

w=angular_freq()
def RCL_System(ak,bk,ck,dk,su_diag,sy_diag,folder_path):
    su2=np.linalg.inv(su_diag)
    akr=ak
    bkr=bk @ sy_diag
    ckr=su2 @ ck
    dkr=dk

    file_path = os.path.join(folder_path, 'real_controller_data.py')

    # Write the code to the file
    with open(file_path, 'w') as f:
        f.write('import numpy as np\n')
        f.write(f'akr = np.array({akr.tolist()})\n')
        f.write(f'bkr = np.array({bkr.tolist()})\n')
        f.write(f'ckr = np.array({ckr.tolist()})\n')
        f.write(f'dkr = np.array({dkr.tolist()})\n')
        f.write('print("AKR:", akr)\n')
        f.write('print("BKR:", bkr)\n')
        f.write('print("CKR:", ckr)\n')
        f.write('print("DKR:", dkr)\n')
    
    print('akr',akr)
    print('bkr',bkr)
    print('ckr',ckr)
    print('dkr',dkr)
    return akr,bkr,ckr,dkr

def Rsystem_feedback(akr,bkr,ckr,dkr,ap0, bp0, cp0, dp0):
    sysKR=control.ss(akr, bkr, ckr, dkr) # Kr
    sysGR = control.ss(ap0, bp0, cp0, dp0) # Gr

    Tsys=control.feedback(sysGR*sysKR, sys2=np.eye((3)), sign=-1)  #K*G
    return Tsys

def Rprint_Tsys(Tsys):
    print('\nTsys:')
    print(Tsys)

def Rplot_FR_Tsys(Tsys,w,output_folder):
    print("\nClose the image to continue the simulation")

    sv_t = triv_sigma(Tsys, w)

    plt.figure()
    plt.semilogx(w, 20*np.log10(sv_t), label=r'$\sigma_1(S_1)$')
    plt.grid(True, which='both', linestyle='--', linewidth=0.5)
    plt.xlim([1e-2, 1e3])
    plt.ylabel('Singular Values [dB]')
    plt.xlabel('Frequency [rad/s]')
    plt.title(' Complementary sensitivity (T) closed loop')
    output_path = os.path.join(output_folder, '19_Complementary sensitivity (T) closed loop.png')
    plt.savefig(output_path)

def SR_Tsys_1(Tsys,output_folder):
    t = np.linspace(0, 20, num=2000)  # Time range from 0 to 20 seconds with 2000 integration points

    # Simulates the system with the new time range and number of integration points
    t, y = step_response(Tsys, T=t)

    plt.figure(figsize=(12, 6))
    plt.subplot(2, 2, 1)  # 2 rows, 2 columns, first subgraph
    plt.plot(t,y[0,0,:],label='u [m/s]')
    plt.plot(t,y[1,0,:],label='v [m/s]')
    plt.plot(t,y[2,0,:],label='r [rad/s]')
    plt.grid()
    plt.xlim([0, 15])
    plt.ylabel('Amplitude')
    plt.xlabel('Time [s]')
    plt.legend(['u [m/s]', 'v [m/s]', 'r [rad/s]'])
    plt.title('Step response in closed loop caused by input 1')

    plt.subplot(2, 2, 2)  # 2 rows, 2 columns, second subgraph
    plt.plot(t,y[0,1,:],label='u [m/s]')
    plt.plot(t,y[1,1,:],label='v [m/s]')
    plt.plot(t,y[2,1,:],label='r [rad/s]')
    plt.grid()
    plt.xlim([0, 15])
    plt.ylabel('Amplitude')
    plt.xlabel('Time [s]')
    plt.legend(['u [m/s]', 'v [m/s]', 'r [rad/s]'])
    plt.title('Step response in closed loop caused by input 2')
    #plt.show()

    plt.subplot(2, 2, 3)  # 2 rows, 2 column, third subgraph
    plt.plot(t,y[0,2,:], label='u [m/s]')
    plt.plot(t,y[1,2,:], label='v [m/s]')
    plt.plot(t,y[2,2,:], label='r [rad/s]')
    plt.grid()
    plt.xlim([0, 15])
    plt.ylabel('Amplitude')
    plt.xlabel('Time [s]')
    plt.legend(['u [m/s]', 'v [m/s]', 'r [rad/s]'])
    plt.title('Step response in closed loop caused by input 3')
    plt.tight_layout()  # Adjusts subgraphs so that they do not overlap

    output_path = os.path.join(output_folder, '20_Step Response in closed loop by input 1,2,3.png')
    plt.savefig(output_path)
    
    plt.show()