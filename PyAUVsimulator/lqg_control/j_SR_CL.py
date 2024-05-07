# 12_
import os
import numpy as np
import matplotlib.pyplot as plt
import control
from control import step_response

#***************************************************************************
#Step response in closed loop
def grafics_SR_CL(al,bl,cl,nc,output_folder):
    sys11=control.ss(al-bl @ cl, bl, cl, np.zeros((nc,nc)))  #OL_CL

    t, y = step_response(sys11)
    print("\nClose the image to continue the simulation")
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
    plt.title('Response in Closed Loop caused by input 1')
    
    plt.subplot(2, 2, 2)  # 2 rows, 2 columns, second subgraph
    plt.plot(t,y[0,1,:],label='u [m/s]')
    plt.plot(t,y[1,1,:],label='v [m/s]')
    plt.plot(t,y[2,1,:],label='r [rad/s]')
    plt.grid()
    plt.xlim([0, 15])
    plt.ylabel('Amplitude')
    plt.xlabel('Time [s]')
    plt.legend(['u [m/s]', 'v [m/s]', 'r [rad/s]'])
    plt.title('Response in Closed Loop caused by input 2')
    #plt.show()

    plt.subplot(2, 2, 3)  # 2 rows, 2 columns, third subgraph
    plt.plot(t,y[0,2,:], label='u [m/s]')
    plt.plot(t,y[1,2,:], label='v [m/s]')
    plt.plot(t,y[2,2,:], label='r [rad/s]')
    plt.grid()
    plt.xlim([0, 15])
    plt.ylabel('Amplitude')
    plt.xlabel('Time [s]')
    plt.legend(['u [m/s]', 'v [m/s]', 'r [rad/s]'])
    plt.title('Response in Closed Loop caused by input 3')
    plt.tight_layout()  # Adjusts subgraphs so that they do not overlap

    output_path = os.path.join(output_folder, '14_Step Response in Closed Loop by input 1,2,3.png')
    plt.savefig(output_path)
    plt.show()

