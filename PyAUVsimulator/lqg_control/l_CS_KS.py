# 14 
import os
import numpy as np
import matplotlib.pyplot as plt
import control
from control import step_response
from lib.constants_funtions import triv_sigma, angular_freq
w=angular_freq()
#***************************************************************************
# Controller
def Def_U(sys8,al,bl,cl,nc):
    sys10=control.ss(al-bl @ cl, bl, -cl, np.eye((nc)))  #S
    # U =K*S
    U=sys8*sys10
    return U

def SR_U(U,output_folder):
    t = np.linspace(0, 20, num=2000)  # Time range from 0 to 20 seconds with 2000 integration points

    # Simulates the system with the new time range and number of integration points
    t, y = step_response(U, T=t)

    print("\nClose the image to continue the simulation")
    plt.figure(figsize=(12, 6))
    plt.subplot(2, 2, 1)  # 2 rows, 2 columns, first subgraph
    plt.plot(t,y[0,0,:],label='Tp1 [N]')
    plt.plot(t,y[1,0,:],label='Tp2 [N]')
    plt.plot(t,y[2,0,:],label='Tp3 [N]')
    plt.grid()
    plt.xlim([0, 15])
    plt.ylabel('Amplitude')
    plt.xlabel('Time [s]')
    plt.legend(['Tp1 [N]', 'Tp2 [N]', 'Tp3 [N]'])
    plt.title('Control step response caused by input 1')
    
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

    plt.subplot(2, 2, 3)  # 2 rows, 2 column, third subgraph
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

    output_path = os.path.join(output_folder, '17_Control step response in Closed Loop by input 1,2,3.png')
    plt.savefig(output_path)
    
    
# Controller Sensitivity K*S
def SC_KS(U,w,output_folder):
    svu = triv_sigma(U, w)
    plt.figure()
    plt.semilogx(w, 20*np.log10(svu), label=r'$\sigma_1(S_1)$')
    plt.grid(True, which='both', linestyle='--', linewidth=0.5)
    plt.xlim([1e-2, 1e3])
    plt.ylabel('Singular Values [dB]')
    plt.xlabel('Frequency [rad/s]')
    #plt.legend()
    plt.title(' Sensitivity (KS)')
    output_path = os.path.join(output_folder, '18_Sensitivity (KS).png')
    plt.savefig(output_path)
    plt.show()