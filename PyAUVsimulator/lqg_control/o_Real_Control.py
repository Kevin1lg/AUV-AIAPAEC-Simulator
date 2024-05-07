# 17
import os
import numpy as np
import matplotlib.pyplot as plt
import control
from control import step_response
from lib.constants_funtions import triv_sigma, angular_freq

w=angular_freq()

#*************************************************************
#*******Real control
#***************************************************************************
# Controller
def Controll_FR(alr,blr,clr,akr,bkr,ckr,dkr,nc):
    sysSr=control.ss(alr-blr @ clr, blr, -clr, np.eye((nc))) # Sr
    sysc=control.ss(akr, bkr, ckr, dkr) # Kr
    # U =K*S
    Ur=sysc*sysSr
    return Ur

def SR_Ur(Ur,output_folder):
    # t, y = step_response(U)
    t = np.linspace(0, 20, num=2000)  # Time range from 0 to 20 seconds with 2000 integration points
    # Simulates the system with the new time range and number of integration points
    t, y = step_response(Ur, T=t)

    print("\nClose the image to continue the simulation")
    plt.figure(figsize=(12, 6))
    plt.subplot(2, 2, 1)  #2 rows, 2 columns, first subgraph
    plt.plot(t,y[0,0,:],label='Tp1 [N]')
    plt.plot(t,y[1,0,:],label='Tp2 [N]')
    plt.plot(t,y[2,0,:],label='Tp3 [N]')
    plt.grid()
    plt.xlim([0, 15])
    plt.ylabel('Amplitude')
    plt.xlabel('Time [s]')
    plt.legend(['Tp1 [N]', 'Tp2 [N]', 'Tp3 [N]'])
    plt.title('Control step response caused by input 1')
    
    #plt.show()

    plt.subplot(2, 2, 2)  # 2 rows, 2 columns, second subgraph
    plt.plot(t,y[0,1,:],label='Tp1 [N]')
    plt.plot(t,y[1,1,:],label='Tp2 [N]')
    plt.plot(t,y[2,1,:],label='Tp3 [N]')
    plt.grid()
    plt.xlim([0, 15])
    plt.ylabel('Amplitude')
    plt.xlabel('Time [s]')
    plt.legend(['Tp1 [N]', 'Tp2 [N]', 'Tp3 [N]'])
    plt.title('Control step response caused by input 2')
    #plt.show()

    plt.subplot(2, 2, 3)  # 2 rows, 2 columns, third subchart
    plt.plot(t,y[0,2,:], label='Tp1 [N]')
    plt.plot(t,y[1,2,:], label='Tp2 [N]')
    plt.plot(t,y[2,2,:], label='Tp3 [N]')
    plt.grid()
    plt.xlim([0, 15])
    plt.ylabel('Amplitude')
    plt.xlabel('Time [s]')
    plt.legend(['Tp1 [N]', 'Tp2 [N]', 'Tp3 [N]'])
    plt.title('Control step response caused by input 3')
    plt.tight_layout()  # Adjusts subgraphs so that they do not overlap

    output_path = os.path.join(output_folder, '25_Control step response of the Real System by input.png')
    plt.savefig(output_path)
    
    #plt.show()

def SC_KS_UR(Ur,w,output_folder):

    svur = triv_sigma(Ur, w)
    
    plt.figure()
    plt.semilogx(w, 20*np.log10(svur), label=r'$\sigma_1(S_1)$')
    plt.grid(True, which='both', linestyle='--', linewidth=0.5)
    plt.xlim([1e-2, 1e3])
    plt.ylabel('Singular Values [dB]')
    plt.xlabel('Frequency [rad/s]')
    #plt.legend()
    plt.title('Control sensitivity (KS)')
    output_path = os.path.join(output_folder, '26_Control sensitivity.png')
    plt.savefig(output_path)
    plt.show()