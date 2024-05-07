# 7_
import os
import numpy as np
import matplotlib.pyplot as plt
import control
from lib.constants_funtions import triv_sigma, angular_freq

w=angular_freq()
#***************************************************************************
#Design of target loop singular values using Kalman Filter

def FK_c(ap,bp,cp,dp):
    ll=-np.linalg.inv(cp @ np.linalg.inv(-ap) @ bp + dp) # Choose ll and lh to match singular values at all frequencies
    lh=-np.linalg.inv(ap) @ bp @ ll
    l=np.block([
        [ll],
        [lh]
    ]) #ll, lh - for low and high frequency loop shaping
    return ll,lh,l

def Print_constants_FK(ll,lh,l):
    print('\nMatrix  ll')
    print(ll)
    print('\nMatrix  lh')
    print(lh)
    print('\nMatrix  l')
    print(l)

def FR_OpenLoop_sys(a,l,c,d,w):
    sys3 = control.ss(a, l, c, d)
    sv3 = triv_sigma(sys3, w)
    return sv3

#***************************************************************************
def constants_NI(nc):
    pnint=np.identity(nc) # Process noise intensity matrix
    mu=0.01 #Measurement noise intesity; used to adjust Kalman Filter
    mnint=mu*np.identity(nc) #Measurement noise intensity matrix
    return pnint,mu,mnint

def Print_NI(pnint,mnint):
    print('\npnint')
    print(pnint)
    print('\nmnint')
    print(mnint)

def care_funtion(a,c,l,mnint):
    # Calculate g1 using the care function
    sig, pole, g1 = control.care(a.T, c.T, np.dot(l, l.T), mnint)

    # Alternate Method for Computing h
    # Calculate h as the transpose of g1
    h = g1.T
    return g1,h

def FR_sys4(a,h,c,d,w):
    sys4 = control.ss(a, h, c, d)
    sv4 = triv_sigma(sys4, w)
    tsv=20*np.log10(sv4)
    return tsv

#***************************************************************************
#GRAPHICS
def grafic_FR(tsv,sv3,w,output_folder):
    plt.figure(figsize=(6,6))
    plt.subplot(2, 1, 1)  # 2 row, 1 columns, first subchart
    print("\nClose the image to continue the simulation")
    plt.semilogx(w, 20*np.log10(sv3), label=r'$\sigma_1(S_1)$')
    plt.grid(True, which='both', linestyle='--', linewidth=0.5)
    # plt.ylim([-200, 100])  
    plt.ylabel('Singular Values [dB]')
    plt.xlim([1e-2, 1e3])
    plt.xlabel('Frequency [rad/s]')
    # plt.legend()
    plt.title('Filter open loop')

    plt.subplot(2, 1, 2)  # 2 row, 1 columns, second subchart
    plt.semilogx(w, tsv, label=r'$\sigma_1(S_1)$')
    plt.grid(True, which='both', linestyle='--', linewidth=0.5)
    plt.ylabel('Singular Values [dB]')
    plt.xlim([1e-2, 1e3])
    plt.xlabel('Frequency [rad/s]')
    plt.title('Target loop')
    plt.tight_layout()  # Adjusts subgraphs so that they do not overlap
    
    output_path = os.path.join(output_folder, '5_Filter open loop and target loop.png')
    plt.savefig(output_path)

    
    plt.show()
    #***************************************************************************
