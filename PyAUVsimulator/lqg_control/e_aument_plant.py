# 6
import os
import numpy as np
import matplotlib.pyplot as plt
import control
from lib.constants_funtions import triv_sigma

# Augment Plant with Integrators at Plant Input and Plot Singular Values
def shapebp(bp):
    ns, nc = bp.shape
    return ns, nc

def PA_abcd(ap,bp,cp,nc,ns):
    a=np.block([
        [np.zeros((nc,ns)), np.zeros((nc, nc))],
        [bp, ap]
    ])

    b = np.block([
        [np.identity(nc)],
        [np.zeros((ns, nc))]
        ])

    c=np.block([
        [np.zeros((nc, nc)), cp]
    ])
    d=np.zeros((nc, nc))
    return a, b, c, d

def PrintPA(a,b,c,d):
    print('\nMatrix  a')
    print(a)
    print('\nMatrix  b')
    print(b)
    print('\nMatrix  c')
    print(c)
    print('\nMatrix  d')
    print(d)

def FR_PA(a,b,c,d,w,output_folder):
    sys2 = control.ss(a, b, c, d)
    sv2 = triv_sigma(sys2, w)

    plt.figure()
    print("\n Close the image to continue the simulation")
    plt.semilogx(w, 20*np.log10(sv2), label=r'$\sigma_1(S_1)$')
    plt.grid(True, which='both', linestyle='--', linewidth=0.5)
    #plt.ylim([-200, 100])
    plt.ylabel('Singular Values [dB]')
    plt.xlim([1e-2, 1e3])
    plt.xlabel('Frequency [rad/s]')
    #plt.legend()
    plt.title('Frequency response of augmented plant')
    output_path = os.path.join(output_folder, '4_Frequency response of augmented plant.png')
    plt.savefig(output_path)
    plt.show()

