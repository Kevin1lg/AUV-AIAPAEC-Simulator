# 13_
#--ANOTHER WAY TO SIMULATE THE CLOSED-LOOP SYSTEM 
import os
import numpy as np
import matplotlib.pyplot as plt
import control
from control import step_response
from lib.constants_funtions import triv_sigma, angular_freq

w=angular_freq()
def ACL_System(ak,bk,ck,dk,ap, bp, cp, dp):
    sys8=control.ss(ak, bk, ck, dk) #K
    sys1 = control.ss(ap, bp, cp, dp) #G
    Tsys=control.feedback(sys1*sys8, sys2=np.eye((3)),sign=-1)
    return sys8, sys1, Tsys

def plot_FR_Tsys(Tsys,w,output_folder):
    sv_t = triv_sigma(Tsys, w)
    plt.figure()
    plt.semilogx(w, 20*np.log10(sv_t), label=r'$\sigma_1(S_1)$')
    plt.grid(True, which='both', linestyle='--', linewidth=0.5)
    plt.xlim([1e-2, 1e3])
    plt.ylabel('Singular Values [dB]')
    plt.xlabel('Frequency [rad/s]')
    #plt.legend()
    plt.title('Complementary sensitivity in closed loop (with "feedback" funtion)')
    output_path = os.path.join(output_folder, '15_Complementary sensitivity in closed loop (with "feedback" funtion).png')
    plt.savefig(output_path)
    #plt.show()

def SR_Tsys(Tsys, output_folder):
    t = np.linspace(0, 20, num=2000)  # Time range from 0 to 20 seconds with 2000 integration points

    # Simulate the system with the new time range and number of integration points
    t, y = step_response(Tsys, T=t)

    print("\nClose the image to continue the simulation")
    plt.figure(figsize=(12, 6))
    plt.subplot(2, 2, 1)  # 2 rows, 2 columns, first subplot
    plt.plot(t, y[0, 0, :], label='u [m/s]')
    plt.plot(t, y[1, 0, :], label='v [m/s]')
    plt.plot(t, y[2, 0, :], label='r [rad/s]')
    plt.grid()
    plt.xlim([0, 15])
    plt.ylabel('Amplitude')
    plt.xlabel('Time [s]')
    plt.legend(['u [m/s]', 'v [m/s]', 'r [rad/s]'])
    plt.title('Response in closed loop caused by input 1')

    plt.subplot(2, 2, 2)  # 2 rows, 2 columns, second subplot
    plt.plot(t, y[0, 1, :], label='u [m/s]')
    plt.plot(t, y[1, 1, :], label='v [m/s]')
    plt.plot(t, y[2, 1, :], label='r [rad/s]')
    plt.grid()
    plt.xlim([0, 15])
    plt.ylabel('Amplitude')
    plt.xlabel('Time [s]')
    plt.legend(['u [m/s]', 'v [m/s]', 'r [rad/s]'])
    plt.title('Response in closed loop caused by input 2')

    plt.subplot(2, 2, 3)  # 2 rows, 2 columns, third subplot
    plt.plot(t, y[0, 2, :], label='u [m/s]')
    plt.plot(t, y[1, 2, :], label='v [m/s]')
    plt.plot(t, y[2, 2, :], label='r [rad/s]')
    plt.grid()
    plt.xlim([0, 15])
    plt.ylabel('Amplitude')
    plt.xlabel('Time [s]')
    plt.legend(['u [m/s]', 'v [m/s]', 'r [rad/s]'])
    plt.title('Response in closed loop caused by input 3')
    plt.tight_layout()  # Adjust subplots so they do not overlap
    output_path = os.path.join(output_folder,'16_Step Response in closed loop by input 1,2,3 (with "feedback" funtion).png')
    plt.savefig(output_path)
    plt.show()  
    