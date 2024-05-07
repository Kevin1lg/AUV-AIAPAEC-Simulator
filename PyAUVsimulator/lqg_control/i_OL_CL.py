# 10_
import os
import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import eig
import control
from lib.constants_funtions import triv_sigma

#***************************************************************************
    #Open Loop Analysis
def open_loop_analysis(ap,bp,cp,ak,bk,ck,nc,ns):
    al=np.block([
        [ap, bp @ ck],
        [np.zeros((ns+nc+nc,ns)),ak]
    ])
    bl=np.block([
        [np.zeros((ns,nc))],
        [bk]
    ])
    cl=np.block([
        [cp, np.zeros((nc,ns+nc+nc))]])
    dl=np.zeros((3,3))
    return al,bl,cl,dl

def printOL(al,bl,cl,dl):
    print('\nMatrix  al')
    print(al)
    print('\nMatrix  bl')
    print(bl)
    print('\nMatrix  cl')
    print(cl)
    print('\nMatrix  dl')
    print(dl)

def graf_OL(al,bl,cl,w,nc,tsv,output_folder):
    sys9=control.ss(al, bl, cl, np.zeros((nc,nc)))
    #
    # plt.figure()
    olpoles, olzeros= control.pzmap(sys9) # Open Loop Zeros
    output_path = os.path.join(output_folder, '11_Open loop zeros.png')
    plt.savefig(output_path)
    print('\nolpoles')
    print(olpoles)
    print('\nolzeros')
    print(olzeros)


    sv9 = triv_sigma(sys9, w)
    plt.figure()
    print("\nClose the image to continue the simulation")
    plt.semilogx(w, 20*np.log10(sv9), label=r'$\sigma_1(S_1)$')
    plt.semilogx(w, tsv, label=r'$\sigma_1(S_1)$')
    plt.grid(True, which='both', linestyle='--', linewidth=0.5)
    plt.xlim([1e-2, 1e3])
    plt.ylabel('Singular Values [dB]')
    plt.xlabel('Frequency [rad/s]')
    plt.title('Open loop frequency response (GK)')
    output_path = os.path.join(output_folder, '12_Open loop frequency response (GK).png')
    plt.savefig(output_path)
    plt.show()

# 11
#***************************************************************************
# Closed Loop Analysis
def poles_cl_(al,bl,cl,a,b,c,h,g):
    clpoles,_= eig(al-bl @ cl)  # Closed Loop Poles
    clpkf,_= eig(a - h @ c)     # Closed Loop Poles Due to Kalman Filter
    clpreg,_= eig(a - b @ g)    # Closed Loop Poles Due to Regulator

    print('\nclpoles')
    print(clpoles)
    print('\nclpkf')
    print(clpkf)
    print('\nclpreg')
    print(clpreg)


def grafic_cl(al,bl,cl,w,nc,output_folder):
    sys10=control.ss(al-bl @ cl, bl, -cl, np.eye((nc)))
    sv10 = triv_sigma(sys10, w)
    print("\nClose the image to continue the simulation")
    plt.figure(figsize=(6,6))
    plt.subplot(2, 1, 1)  # 2 rows, 1 columns, first subgraph
    plt.semilogx(w, 20*np.log10(sv10), label=r'\\sigma_1(S_1)')
    plt.grid(True, which='both', linestyle='--', linewidth=0.5)
    plt.ylabel('Singular values [dB]')
    plt.xlim([1e-2, 1e3])
    plt.xlabel('Frequency [rad/s]')
    #plt.legend()
    plt.title('Sensitivity(S)')
    

    sys11=control.ss(al-bl @ cl, bl, cl, np.zeros((nc,nc)))
    sv11 = triv_sigma(sys11, w)
    plt.subplot(2, 1, 2)  # 2 rows, 1 columns, second subgraph
    plt.semilogx(w, 20*np.log10(sv11), label=r'$\sigma_1(S_1)$')
    plt.grid(True, which='both', linestyle='--', linewidth=0.5)
    plt.xlim([1e-2, 1e3])
    plt.ylabel('Singular Values [dB]')
    plt.xlabel('Frequency [rad/s]')
    #plt.legend()
    plt.title('Complementary sensitivity (T)')
    plt.tight_layout()  # Adjusts subgraphs so that they do not overlap

    output_path = os.path.join(output_folder, '13_Sensitivity(S) and Complementary sensitivity in closed loop (T).png')
    plt.savefig(output_path)
    
    plt.show()