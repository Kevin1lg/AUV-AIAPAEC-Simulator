import os
from scipy import signal
from lib.main_loop import Espace

def discretize(akr, bkr, ckr, dkr,Ts,folder_path):
    # Define the sampling frequency
    Espace()
    print('Tiempo de muestreo',Ts)
    Espace()
    # Create the state space system
    sys = signal.StateSpace(akr, bkr, ckr, dkr)

    # Convert the system to discrete time
    sys_d = signal.cont2discrete((sys.A, sys.B, sys.C, sys.D), Ts)

    akr_d1 = sys_d[0]
    bkr_d1 = sys_d[1]
    ckr_d1 = sys_d[2]
    dkr_d1 = sys_d[3]

    file_path = os.path.join(folder_path,f"discrete_control_Ts_{Ts}.py")

    # Write the code to the file
    with open(file_path, 'w') as f:
        f.write('import numpy as np\n')
        f.write(f'#Ts={Ts}\n')
        f.write(f'AK = np.array({akr_d1.tolist()})\n')
        f.write(f'BK = np.array({bkr_d1.tolist()})\n')
        f.write(f'CK = np.array({ckr_d1.tolist()})\n')
        f.write(f'DK = np.array({dkr_d1.tolist()})\n')
        f.write('print("AK:", AK)\n')
        f.write('print("BK:", BK)\n')
        f.write('print("CK:", CK)\n')
        f.write('print("DK:", DK)\n')
    
    print(f'Generate a discrete real control code: discrete_control_Ts_{Ts}')
    print('in the folder real_state_space_and_discretized_control')


