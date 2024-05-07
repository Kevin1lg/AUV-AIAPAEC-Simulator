# 3
import os
import numpy as np
import matplotlib.pyplot as plt
import control
from lib.constants_funtions import triv_sigma
#***************************************************************************
# FREQUENCY RESPONSE: Unscaled Singular Values

# Create the state space system
# Dynamic system

#***************************************************************************
def RF_USV(ap0,bp0,cp0,dp0,w):
    sys = control.ss(ap0, bp0, cp0, dp0)
    sv = triv_sigma(sys, w)
    return sv

def plot_RF(sv,w,output_folder):
    plt.figure()
    print("\nClose the image to continue the simulation")
    plt.semilogx(w, 20*np.log10(sv), label=r'$\sigma_1(S_1)$')
    plt.grid(True, which='both', linestyle='--', linewidth=0.5)
    #plt.ylim([-120, 20])
    plt.ylabel('Singular values [dB]')
    plt.xlim([1e-2, 1e3])
    plt.xlabel('Frequency [rad/s]')
    plt.title('Frequency response of unscaled plant')

    output_path = os.path.join(output_folder, '2_Frequency response of unscaled plant.png')
    plt.savefig(output_path)
    plt.show()

# 5
#***************************************************************************
# FREQUENCY RESPONSE: Scaled Singular Values

def FR_SSV(ap,bp,cp,dp,w,output_folder):
    sys1 = control.ss(ap, bp, cp, dp)
    sv1 = triv_sigma(sys1, w)

    plt.figure()
    print("\nClose the image to continue the simulation")
    plt.semilogx(w, 20*np.log10(sv1), label=r'$\sigma_1(S_1)$')
    plt.grid(True, which='both', linestyle='--', linewidth=0.5)
    #plt.ylim([-100, 40])
    plt.ylabel('Singular Values [dB]')
    plt.xlim([1e-2, 1e3])
    plt.xlabel('Frequency [rad/s]')
    #plt.legend()
    plt.title('Frequency response of scaled plant')

    output_path = os.path.join(output_folder, '3_Frequency response of scaled plant.png')
    plt.savefig(output_path)
    plt.show()