# 8_
import os
import numpy as np
import matplotlib.pyplot as plt
import control

from scipy.linalg import eig
from lib.constants_funtions import triv_sigma,angular_freq

w=angular_freq()
#***************************************************************************
def poles_OL(a,h,c,nc,w,output_folder):
    tolpoles,_ = eig(a) # Target open loop poles
    sys5 = control.ss(a - h @ c, h, -c, np.eye((nc)))
    #plt.figure()
    targpoles,targzeros = control.pzmap(sys5,plot=None, grid=None, title='Pole zero map') 
    
    output_path = os.path.join(output_folder, '6_Target open loop zeros.png')
    plt.savefig(output_path)    
    
    tcpoles,_=eig(a - h @ c) # Target closed loop poles
    sv5 = triv_sigma(sys5, w)

    print('\ntolpoles')
    print(tolpoles)
    print('\ntargpoles')
    print(targpoles)
    print('\ntargzeros')
    print(targzeros)
    print('\ntcpoles')
    print(tcpoles)

    print("\nClose the image to continue the simulation")
    
    plt.figure(figsize=(6, 6))
    plt.subplot(2, 1, 1)  # 2 filas, 1 columna, primer subgráfico
    plt.semilogx(w, 20*np.log10(sv5), label=r'$\sigma_1(S_1)$')
    plt.grid(True, which='both', linestyle='--', linewidth=0.5)
    #plt.ylim([-200, 100])
    plt.ylabel('Singular Values [dB]')
    plt.xlim([1e-2, 1e3])
    plt.xlabel('Frequency [rad/s]')
    plt.title('Target sensitivity (S)')
    

    sys6 = control.ss(a - h @ c, h, c, np.zeros((nc,nc)))
    sv6 = triv_sigma(sys6, w)
    
    plt.subplot(2, 1, 2)  # 2 rows, 1 columns, second subgraph
    plt.semilogx(w, 20*np.log10(sv6), label=r'$\sigma_1(S_1)$')
    plt.semilogx(w, 20*np.log10(10/w), label=r'$\sigma_1(S_1)$')
    plt.grid(True, which='both', linestyle='--', linewidth=0.5)
    plt.ylabel('Singular Values [dB]')
    plt.xlim([1e-2, 1e3])
    plt.xlabel('Frequency [rad/s]')
    plt.title('Target sensitivity (S) and target loop')
    plt.tight_layout()  # Adjusts subgraphs so that they do not overlap

    output_path = os.path.join(output_folder, '7_Target sensitivity (S) and target loop.png')
    plt.savefig(output_path)
    plt.show()
