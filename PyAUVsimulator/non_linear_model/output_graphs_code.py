import os
import matplotlib.pyplot as plt
from scipy.integrate import odeint
from non_linear_model import model_auv
from lib.constants import constants_initial
from lib.constants_funtions import *

Gb, Inertia, datos_ext, dimensions, Parameters_water, dot, Xx, Yy, Nn= constants_initial()
x0=x0_c()
Ni,t=Ni_c()

def out_graphs(x0, t, Gb, Inertia, datos_ext, dimensions,Parameters_water, dot, Xx, Yy, Nn,
                distance_propellers,Tp1, Tp2, Tp3,output_folder):
    
    # Call odeint with the aiapaec function and the necessary parameters.
    x = odeint(model_auv, x0, t, args=(Gb, Inertia, datos_ext, dimensions, Parameters_water, dot, Xx, Yy, Nn, distance_propellers,Tp1, Tp2, Tp3))
    
    print('System outputs with respect to time')
    print("\nClose the images to continue with the simulation")
    
    plt.figure(figsize=(12,6))
    
    plt.subplot(2, 3, 1)  # 2 rows, 3 columns, first subgraph
    plt.plot(t, x[:, 0], c='b', label='u')
    plt.xlabel('Time [s]')
    plt.ylabel('Vel.[m/s] ')
    plt.grid()
    plt.legend(loc='upper right')
    plt.title('Velocity u')

    plt.subplot(2, 3, 2)  # 2 rows, 3 columns, second subgraph
    plt.plot(t, x[:, 1], c='r', label='v')
    plt.xlabel('Time [s] ')
    plt.ylabel('Vel.[m/s] ')
    plt.grid()
    plt.legend(loc='upper right')
    plt.title('Velocity v')

    plt.subplot(2, 3, 3)  # 2 rows, 3 columns, third subgraph
    plt.plot(t, x[:, 2], c='g', label='r')
    plt.xlabel('Time [s] ')
    plt.ylabel('Vel. Angular [rad/s] ')
    plt.grid()
    plt.legend(loc='upper right')
    plt.title('Velocity r')

    plt.subplot(2, 3, 4)  # 2 rows, 3 columns, fourth subgraph
    plt.plot(t, x[:, 3], c='y', label='xpos')
    plt.xlabel('Time [s] ')
    plt.ylabel('Pos. [m] ')
    plt.grid()
    plt.legend(loc='upper right')
    plt.title('Position on x-axis')

    plt.subplot(2, 3, 5)  # 2 rows, 3 columns, fifth subgraph
    plt.plot(t, x[:, 4], c='m', label='ypos')
    plt.xlabel('Time [s] ')
    plt.ylabel('Pos. [m] ')
    plt.grid()
    plt.legend(loc='upper right')
    plt.title('Position on y-axis')

    plt.subplot(2, 3, 6)  # 2 rows, 3 columns, sixth subgraph
    plt.plot(t, x[:, 5], c='c', label='apsi')
    plt.xlabel('Time [s] ')
    plt.ylabel('Yaw [°] ')
    plt.grid()
    plt.legend(loc='upper right')
    plt.title('[°]  Yaw')
    plt.tight_layout()   # Adjust the subgraphs so that they do not overlap.

    output_path = os.path.join(output_folder, 'Non-Linear Outputs')
    plt.savefig(output_path)
    #plt.show()
   
    plt.figure()
    plt.plot(x[:, 3], x[:, 4], c='c', label='XYpos')
    plt.xlabel('Pos. X [m] ')
    plt.ylabel('Pos. Y [m] ')
    plt.grid()
    plt.legend(loc='upper right')

    plt.tight_layout()   # Adjust the subgraphs so that they do not overlap.

    output_path = os.path.join(output_folder, 'Position in the x, y plane')
    plt.savefig(output_path)
    
    
    plt.show()
