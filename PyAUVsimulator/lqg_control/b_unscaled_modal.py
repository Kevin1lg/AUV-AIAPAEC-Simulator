# 2_
import os
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import lti, lsim

#***************************************************************************
# Unscaled Modal Analysis: We want to examine the natural modes (tendencies)
# Define the time vector
def unscaled_model(ap0,bp0,evec):
    
    t = np.arange(0, 200.1, 0.1)
    # Set input u to zero for all time in order to generate zero input response;
    u = np.zeros((len(t), 3))

    system = lti(ap0, bp0, np.eye(3,3), np.zeros((3, 3)))

    # Define the vector of initial values ​​'x0'
    x0 = np.array(evec[:, 0])  ## Adjust 'x0' according to your needs
    x1 = np.array(evec[:, 1])  ## Adjust 'x1' according to your needs
    x2 = np.array(evec[:, 2])  ## Adjust 'x2' according to your needs

    # Calculate the Time response of the system to the input system 'u' with initial values ​​'x0'
    t, y, _ = lsim(system, u, t, X0=x0)     # Here we specify X0 as 'x0'
    t1, y1, _ = lsim(system, u, t, X0=x1)   # Here we specify X0 as 'x1'
    t2, y2, _ = lsim(system, u, t, X0=x2)   # Here we specify X0 as 'x2'
    
    return t, y, t1, y1, t2, y2, x0, x1, x2

def plot_um(t, y, t1, y1, t2, y2, x0, x1, x2, output_folder):
    
    print("\nClose the image to continue the simulation")
    # Create the chart
    plt.figure(figsize=(12, 6)) 

    plt.subplot(3, 1, 1)  # 3 rows, 1 column, first subchart
    plt.plot(t, y)
    plt.grid()
    plt.xlim([0,200])
    plt.title(f'Time response with initial value x_0 = {x0}')
    plt.xlabel('Time [s]')
    plt.ylabel('States')

    plt.subplot(3, 1, 2)  # 3 rows, 1 column, second subchart
    plt.plot(t1, y1)
    plt.grid()
    plt.xlim([0,200])
    plt.title(f'Time response with initial value x_0 = {x1}')
    plt.xlabel('Time [s]')
    plt.ylabel('States')

    plt.subplot(3, 1, 3)  # 3 rows, 1 column, third subgraph
    plt.plot(t2, y2)
    plt.grid()
    plt.xlim([0,200])
    plt.title(f'Time response with initial value x_0 = {x2}')
    plt.xlabel('Time [s]')
    plt.ylabel('States')
    plt.tight_layout()  # Adjust the subgraphs so that they do not overlap.

    output_path = os.path.join(output_folder, '1_Time response unscaled modal analysis.png')
    plt.savefig(output_path)

    plt.show()
    