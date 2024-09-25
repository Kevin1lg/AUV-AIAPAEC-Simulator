import numpy as np
import datetime
import os
from scipy.integrate import trapz
from constants import *  
from added_mass import * 
from lift_body_flap import *  
from drag import *

# Combined terms
yuv = yuvl + yuvf
yur = yur + yurf
zuw = zuwl + zuwf
zuq = zuq + zuqf
muw = muwa + muwl + muwf
muq = muq + muqf
nuv = nuva + nuvl + nuvf
nur = nur + nurf

# Setting vehicle parameters easily translated by latex table
list1 = ['ya', 'yb', 'yc', 'yd']
list2 = ['m    ', 'rad  ', 'rad/s', 'm/s  ']
data = [2, 3, 5, 1]
n = len(data)
print('The text will appear in tabular latex format')
print('Parameter \t & Value \t & Unit \t & Description \n \\hline \n')
for k in range(n):
    print(f'{list1[k]} \t & {data[k]} \t & {list2[k]} \t & \\\\ \n ')
print('\\hline \n')

# Declaring structures
class Struct:
    def __init__(self, name1, name2, value, unit, description):
        self.name1 = name1
        self.name2 = name2
        self.value = value
        self.unit = unit
        self.description = description

S = [
    Struct('X_{uu}     ', 'xuu', xuu, 'kg/m       ', 'Cross-flow drag'),
    Struct('X_{\dot{u}}', 'xudot', xudot, 'kg         ', 'Added mass'),
    Struct('X_{wq}     ', 'xwq', xwq, 'kg/rad     ', 'Added mass cross-term'),
    Struct('X_{qq}     ', 'xqq', xqq, 'kg m/rad   ', 'Added mass cross-term'),
    Struct('X_{vr}     ', 'xvr', xvr, 'kg/rad     ', 'Added mass cross-term'),
    Struct('X_{rr}     ', 'xrr', xrr, 'kg m/rad   ', 'Added mass cross-term'),
    Struct('Y_{vv}     ', 'yvv', yvv, 'kg/m       ', 'Cross-flow drag'),
    Struct('Y_{rr}     ', 'yrr', yrr, 'kg m/rad^2 ', 'Cross-flow drag'),
    Struct('Y_{uv}     ', 'yuv', yuv, 'kg/m       ', 'Body lift force and fin lift'),
    Struct('Y_{\dot{v}}', 'yvdot', yvdot, 'kg         ', 'Added mass'),
    Struct('Y_{\dot{r}}', 'yrdot', yrdot, 'kg m/rad   ', 'Added mass'),
    Struct('Y_{ur}   ', 'yur', yur, 'kg/rad     ', 'Added mass cross-term and fin lift'),
    Struct('Y_{wp}   ', 'ywp', ywp, 'kg/rad     ', 'Added mass cross-term'),
    Struct('Y_{pq}   ', 'ypq', ypq, 'kg m/rad   ', 'Added mass cross-term'),
    Struct('Y_{uu\delta_r}', 'yuudelr', yuudelr, 'kg/(m rad) ', 'Fin lift force'),
    Struct('Z_{ww}   ', 'zww', zww, 'kg/m       ', 'Cross-flow drag'),
    Struct('Z_{qq}   ', 'zqq', zqq, 'kg m /rad^2', 'Cross-flow drag'),
    Struct('Z_{uw}   ', 'zuw', zuw, 'kg/m       ', 'Body lift force and fin lift'),
    Struct('Z_{\dot{w}}', 'zwdot', zwdot, 'kg         ', 'Added mass'),
    Struct('Z_{\dot{q}}', 'zqdot', zqdot, 'kg m/rad   ', 'Added mass'),
    Struct('Z_{uq}   ', 'zuq', zuq, 'kg/rad     ', 'Added mass cross-term and fin lift'),
    Struct('Z_{vp}   ', 'zvp', zvp, 'kg/rad     ', 'Added mass cross-term'),
    Struct('Z_{rp}   ', 'zrp', zrp, 'kg/rad     ', 'Added mass cross-term'),
    Struct('Z_{uu\delta_s}', 'zuudels', zuudels, 'kg/(m rad)  ', 'Fin lift force'),
    Struct('K_{pp}   ', 'kpp', kpp, 'kg m^2/rad^2', 'Rolling resistance'),
    Struct('K_{\dot{p}}', 'kpdot', kpdot, 'kg m^2/rad  ', 'Added mass'),
    Struct('M_{ww}   ', 'mww', mww, 'kg         ', 'Cross-flow drag'),
    Struct('M_{qq}   ', 'mqq', mqq, 'kg m^2/rad^2', 'Cross-flow drag'),
    Struct('M_{uw}   ', 'muw', muw, 'kg         ', 'Body and fin lift and munk moment'),
    Struct('M_{\dot{w}}', 'mwdot', mwdot, 'kg m       ', 'Added mass'),
    Struct('M_{\dot{q}}', 'mqdot', mqdot, 'kg m^2/rad ', 'Added mass'),
    Struct('M_{uq}   ', 'muq', muq, 'kg m/rad   ', 'Added mass cross-term and fin lift'),
    Struct('M_{vp}   ', 'mvp', mvp, 'kg m/rad   ', 'Added mass cross-term'),
    Struct('M_{rp}   ', 'mrp', mrp, 'kg m^2/rad^2', 'Added mass cross-term'),
    Struct('M_{uu\delta_s}', 'muudels', muudels, 'kg/rad     ', 'Fin lift moment'),
    Struct('N_{vv}   ', 'nvv', nvv, 'kg         ', 'Cross-flow drag'),
    Struct('N_{rr}   ', 'nrr', nrr, 'kg m^2/rad^2', 'Cross-flow drag'),
    Struct('N_{uv}   ', 'nuv', nuv, 'kg         ', 'Body and fin lift and munk moment'),
    Struct('N_{\dot{v}}', 'nvdot', nvdot, 'kg m       ', 'Added mass'),
    Struct('N_{\dot{r}}', 'nrdot', nrdot, 'kg m^2/rad ', 'Added mass'),
    Struct('N_{ur}   ', 'nur', nur, 'kg m/rad   ', 'Added mass cross-term and fin lift'),
    Struct('N_{wp}   ', 'nwp', nwp, 'kg m/rad   ', 'Added mass cross-term'),
    Struct('N_{pq}   ', 'npq', npq, 'kg m^2/rad^2', 'Added mass cross-term'),
    Struct('N_{uu\delta_r}', 'nuudelr', nuudelr, 'kg/rad     ', 'Fin lift moment')
]

#**********************************************************************************
# Name of the folder where the files will be saved
output_folder = 'file_out'
# Create the folder
os.makedirs(output_folder, exist_ok=True)

# Write to files
date_now = datetime.datetime.now().strftime("%Y%m%dT%H%M%S")
file_out_txt = os.path.join(output_folder, f'file_out_{date_now}.txt')
file_out_m = os.path.join(output_folder, f'file_out_{date_now}.m')

with open(file_out_txt, 'w') as fid:
    fid.write('Parameter \t & Value \t & Unit \t & Description\\\\ \n\\hline \n')
    for s in S:
        fid.write(f'${s.name1}$ \t & ${s.value:+1.6e}$ \t & ${s.unit}$ \t & {s.description} \\\\ \n')
    fid.write('\\hline')

with open(file_out_m, 'w') as fid:
    fid.write('%% This is a m file generated by parameter_AIAPAEC.m \n%% All the AUV parameters are expressed here \n')
    for s in S:
        fid.write(f'{s.name2} = {s.value:+1.6e} ; \t %% {s.unit} |\t {s.description} \n')

#**********************************************************************************
# Non-dimensional parameters

r1 = rho
r2 = (1/2) * rho * l**2
r3 = (1/2) * rho * l**3
r4 = (1/2) * rho * l**4
r5 = (1/2) * rho * l**5

rr_vector1 = np.array([
    r2,  # xuu
    r3,
    r3,
    r4,
    r3,
    r4,
    r2,  # yvv
    r4,  # yrr review
    r2,
    r3,
    r4,
    r3,
    r3,
    r4,
    r2,
    r2,  # zww review
    r4,  # zqq review (warning)
    r2,
    r2,
    r4,
    r3,
    r3,
    r4,  # zrp review
    r2,
    r5,  # kpp warning
    r5,
    r3,  # mww
    r5,  # mqq warning! (see Standard eq. of motion for submarine simulation by Gertler, 1967)
    r3,
    r4,
    r5,
    r4,
    r4,  # mvp review
    r5,
    r3,
    r3,  # nvv review
    r5,  # nrr warning!
    r3,
    r4,
    r5,
    r4,
    r4,  # nwp review
    r5,
    r3,
    1,  # Qp
    1   # Qp
])

# Obtain the transpose of vector rr_vector
rr_vector = rr_vector1.T

#**********************************************************************************
file_out_nd_txt = os.path.join(output_folder, f'file_out_nd_{date_now}.txt')
file_out_nd_m = os.path.join(output_folder, f'file_out_nd_{date_now}.m')

# Save TXT file
with open(file_out_nd_txt, 'w') as fid:
    fid.write('Parameter \t & Value \t & Unit \t & Description\\\\ \n\\hline \n')
    for k, s in enumerate(S):
        nd = f"{s.name1[0]}'{s.name1[1:]}"
        fid.write(f'${nd}$ \t & ${s.value/rr_vector[k]:+1.6e}$ \t & {"------"} \t & {s.description} \\\\ \n')
    fid.write('\\hline')

# Save file M
with open(file_out_nd_m, 'w') as fid:
    fid.write('%% This is a m file generated by parameter_AIAPAEC.m \n%% Non-dimensional parameters are expressed here \n')
    for k, s in enumerate(S):
        fid.write(f'{s.name2} = {s.value/rr_vector[k]:+1.6e} ; \t %% {"------"} |\t {s.description} \n')