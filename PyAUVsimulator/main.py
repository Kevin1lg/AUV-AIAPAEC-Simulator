import os
import matplotlib.pyplot as plt
from scipy import signal
from lib import *
from lib.constants import *
from non_linear_model import *
from lqg_control import *

#*******************************************************************************************************
# Create result folders
#*******************************************************************************************************

output_folder_NL = "non_linear_model_image_outputs"
if not os.path.exists(output_folder_NL):
    os.makedirs(output_folder_NL)

output_folder_MC = f"lqg_control_image_outputs"
if not os.path.exists(output_folder_MC):
    os.makedirs(output_folder_MC)

folder_path = "real_state_space_and_discretized_control"
if not os.path.exists(folder_path):
    os.makedirs(folder_path)
#*******************************************************************************************************
# Presentation
#*******************************************************************************************************
Espace()
printInitialInfoNL() 
Espace()
InfoTp_initial()
Espace()

#*******************************************************************************************************
# Adjustment of the propeller distances
#*******************************************************************************************************

def Enter_new_values_dp():
    proceed_with_data = False

    while not proceed_with_data:
        print('Enter the distance between the thrusters P1, P2')
        while True:
            try:
                number1 = float(input("Enter the value: "))
                break
            except ValueError:
                Invalid_number()

        print('Enter the distance between the P3 thruster and the center of gravity')
        while True:
            try:
                number2 = float(input("Enter the value: "))
                break
            except ValueError:
                Invalid_number()
        Espace()
        print(f"Entered:\nDistance between P1, P2: dp1=dp2= {number1},m\nDistance from P3: dp3= {number2},m")
        Espace()
        print('Are these values correct?')
        Resp = input("yes[y] or no[n]: ")
        # Verify user confirmation
        if Resp == "y" or Resp == "Y":
            dp1 = number1  
            dp2 = number1
            dp3 = number2 
            distance_propellers = np.block([dp1, dp2, dp3])
            print("Confirmed. Proceeding with the data.")
            Espace()
            proceed_with_data = True
        elif Resp=="n" or Resp == "N":
            print("Data input canceled. Please re-enter the correct values.")
            Espace()
        else:
            Invalid_option()
            Espace()
    return  distance_propellers

while True:
    # Loop for the assignment of distance measurements of the thrusters
    print('Do you want to change the values between the default thruster distances?')
    Resp = input("yes[y] or no[n]: ")
    Espace()
    # Verify user confirmation
    if Resp == "y" or Resp == "Y":
        distance_propellers=Enter_new_values_dp()
        Espace()
        break 
    elif Resp=="n" or Resp == "N":
        distance_propellers=distance_propel()
        Espace()
        break 
    else:
        Invalid_option()
        Espace()

dp1, dp2, dp3 = distance_propellers
print_dp(dp1,dp2,dp3)

#*******************************************************************************************************
# NON-LINEAR MODEL
#*******************************************************************************************************

def NON_LINEAR_MODEL(output_folder_NL):
    def Simulation_NLM():
        Info_Input_Tp()
        Espace()
        def Model_no_Lineal():
            while True:
                # Enter three numbers of the force Tp1, Tp2, Tp3
                number1 = None
                while number1 is None:
                    try:
                        number1 = int(input("Enter the value for Tp1: "))
                    except ValueError:
                        Invalid_number()

                number2 = None
                while number2 is None:
                    try:
                        number2 = int(input("Enter the value for Tp2: "))
                    except ValueError:
                        Invalid_number()

                number3 = None
                while number3 is None:
                    try:
                        number3 = int(input("Enter the value for Tp3: "))
                    except ValueError:
                        Invalid_number()

                Espace()

                print(f"You entered:\nTp1: {number1}\nTp2: {number2}\nTp3: {number3}")

                Espace()
                print('Are these values correct?')
                Resp = input("yes[y] or no[n]: ")

                # Verify user confirmation
                if Resp == "y" or Resp == "Y":
                    print("Confirmed. Proceeding with the data.")
                    Espace()
                    break 
                elif Resp=="n" or Resp == "N":
                    print("Data input canceled. Please re-enter the correct values.")
                    Espace()
                else:
                    Invalid_option()
                    Espace()

            Tp1, Tp2, Tp3=input_tp(number1,number2,number3)

            print('Final values of Tp1, Tp2, Tp3:  ')
            print('Tp1 = ',Tp1)
            print('Tp2 = ',Tp2)
            print('Tp3 = ',Tp3)
            Espace()
            out_graphs(x0, t, Gb, Inertia, datos_ext, dimensions, Parameters_water, dot, Xx, Yy, Nn, distance_propellers,Tp1, Tp2, Tp3,output_folder_NL)
            Espace()

        Model_no_Lineal()

        while True:
            print('¿You want to see other outputs when changing Tp values?')
            Resp = input("yes [y] or no [n]: ")
            # Verify user confirmation
            if Resp == "y" or Resp == "Y":
                Espace()
                Model_no_Lineal()
            elif Resp=="n" or Resp == "N":
                print("The nonlinear modeling process has been completed.")
                Espace()
                break  
            else:
                Invalid_option()  

    while True:
        print("Do you want to display the output signals of the nonlinearized model? ")
        continuar_simulacion = input("yes[y] or no[n]: ")
        Espace()

        if continuar_simulacion == 'y' or continuar_simulacion == 'Y':
            Simulation_NLM()
            break
        elif continuar_simulacion == 'n' or continuar_simulacion == 'N':
            print(" ")
            break  # Salir del bucle
        else:
            Invalid_option()
            Espace()

#***********************------------------------***********************
NON_LINEAR_MODEL(output_folder_NL)

#*******************************************************************************************************
# MODEL LINEARIZATION
#*******************************************************************************************************

def MODEL_LINEARIZATION():
    print('SYSTEM LINEARIZATION')
    Espace()
    print('The system is then linearized by obtaining the matrices A, B of the linearized model.')
    Espace()
    #***************************************************************************n
    
    t, X0, U0= constants_LINEARIZATION()
    X = np.array(X0, dtype = np.float64)
    U = np.array(U0, dtype = np.float64)
    A=dfA(t, X, U,Gb, Inertia, datos_ext, dimensions, Parameters_water, dot, Xx, Yy, Nn, distance_propellers, aa = 1)
    B=dfB(t, X, U,Gb, Inertia, datos_ext, dimensions, Parameters_water, dot, Xx, Yy, Nn, distance_propellers, aa = 1)
    
    
    #***************************************************************************
    # Linearized model constants
    A,B,C,D = constantes_ABCD(A,B)
    while True:
        print('Do you want to visualize the linearized model constants?')
        Resp = input("yes[y] or no[n]: ")
        Espace()
        if Resp == "y" or Resp == "Y":
            print('Linearization constants (A, B, C, D)')
            Espace()
            printConstsLineal(A,B,C,D)
            break 
        elif Resp=="n" or Resp == "N":
            print('  ')    
            break  
        else:
            Invalid_option()
            Espace() 
    Espace()
    print('Verification if the model is controllable and observable')
    Espace()
    check_ctr_obs(A,B,C,D)
    Espace()
    return A,B,C,D
A,B,C,D=MODEL_LINEARIZATION()
ap0, bp0, cp0, dp0 = constantes_abcdp(A,B,C,D)

#*******************************************************************************************************
#Manually reassigned linearized constants of the model to be controlled following the above suggestions.
#*******************************************************************************************************

while True:
    print('Manually reassigned linearized constants of the model to be controlled following the above suggestions.')
    Espace()
    print('¿Do you want to visualize the Model Constants to be Controlled?')
    Resp = input("yes[y] or no[n]: ")
    Espace()
    if Resp == "y" or Resp == "Y":
        print('Constants to control (ap0,bp0,cp0,dp0)')
        print('The variables to be controlled are velocities u, v and r')
        Espace()
        printConstsControl(ap0,bp0,cp0,dp0)
        break 
    elif Resp=="n" or Resp == "N":
        print(' ')    
        break  
    else:
        Invalid_option()
        Espace()

#*******************************************************************************************************
#*******************************************************************************************************
""" (model_control)
Select the operation regarding to LQG/LTR control.
1. Controllability and observability of the LTI system.
2. LTI system.
3. Frequency response of the LTI system.
4. Scaling of the LTI system.
5. Frequency response of the scaled LTI system.
6. Augmented plant (by integrator).
7. Kalman filter.
8. Poles in open loop.
9. LQR control design.
10. Open loop analysis.
11. Closed loop analysis.
12. Step response in closed loop.
13. Step response in closed loop (with alternative function "feedback").
14. LQG/LTR Controller for the Augmented plant.
15. Plot the responses with the actual controller (augmented and scaled). 
16. Actual open  and closed loop analysis.
17. Plot the actual control signal Tp1, Tp2, and Tp3.
18. Obtain the state space matrices of the controller in discrete time.

"""
#*******************************************************************************************************
#*******************************************************************************************************

Espace()
printInfoCalculosCtrl()
Espace()
def Cltr_no():
    no = None
    while no is None:
        try:
            no = int(input("Select the operation regarding to LQG/LTR control: "))
            Espace()
                        
            if no == 1:
                """1. Controllability and observability of the LTI system."""
                # a_ctrl_obs_eigen
                rcm=Controllability(ap0, bp0, cp0, dp0)
                rom=printObservability(ap0, bp0, cp0, dp0)
                note_ctr_obs(rcm,rom,ap0)
                Espace()
                eval, evec=EingV(ap0)
                printEingV( eval, evec)
                Espace()

            elif no == 2:
                """2. LTI system. """
                print('LTI system.')
                # a_ctrl_obs_eigen
                eval, evec = EingV(ap0)
                # b_unscaled_modal
                t, y, t1, y1, t2, y2, x0, x1, x2= unscaled_model(ap0,bp0,evec)
                plot_um(t, y, t1, y1, t2, y2, x0, x1, x2, output_folder_MC)
                Espace()

            elif no == 3:
                """3. Frequency response of the LTI system."""
                print('Frequency response of the LTI system.')
                # d_Freq_resp
                sv=RF_USV(ap0,bp0,cp0,dp0,w)
                plot_RF(sv,w,output_folder_MC)
                Espace()

            elif no == 4:
                """4. Scaling of the LTI system."""
                print('Scaling of the LTI system.')
                # c_scaling_matrix
                su_diag,sx_diag,sy_diag=scaling_matrix()
                ap,bp,cp,dp=scaled_dinamics(ap0,bp0,cp0,dp0,su_diag, sx_diag, sy_diag)
                print_scaling_matrix(su_diag, sx_diag, sy_diag)
                print_scaled_dinamics(ap,bp,cp,dp)
                Espace()

            elif no == 5:
                """5. Frequency response of the scaled LTI system."""
                print('Frequency response of the scaled LTI system.')
                # c_scaling_matrix
                su_diag,sx_diag,sy_diag=scaling_matrix()
                ap,bp,cp,dp=scaled_dinamics(ap0,bp0,cp0,dp0,su_diag, sx_diag, sy_diag)
                # d_Freq_resp
                FR_SSV(ap,bp,cp,dp,w,output_folder_MC)
                Espace() 

            elif no == 6:
                """6. Augmented plant (by integrator)."""
                print('Augmented plant (by integrator).')
                # c_scaling_matrix
                su_diag,sx_diag,sy_diag=scaling_matrix()
                ap,bp,cp,dp=scaled_dinamics(ap0,bp0,cp0,dp0,su_diag, sx_diag, sy_diag)
                # e_aument_plant
                ns, nc=shapebp(bp)
                a, b, c, d=PA_abcd(ap,bp,cp,nc,ns)  
                PrintPA(a,b,c,d)
                FR_PA(a,b,c,d,w,output_folder_MC)
                Espace()

            elif no == 7:
                """7. Kalman filter."""
                print('Kalman filter.')
                # c_scaling_matrix
                su_diag,sx_diag,sy_diag=scaling_matrix()
                ap,bp,cp,dp=scaled_dinamics(ap0,bp0,cp0,dp0,su_diag, sx_diag, sy_diag)
                # e_aument_plant
                ns, nc=shapebp(bp)
                a, b, c, d=PA_abcd(ap,bp,cp,nc,ns)
                # f_Kalman_Filter
                ll,lh,l=FK_c(ap,bp,cp,dp)
                Print_constants_FK(ll,lh,l)
                sv3=FR_OpenLoop_sys(a,l,c,d,w)

                pnint,mu,mnint=constants_NI(nc)
                Print_NI(pnint,mnint)
                g1,h=care_funtion(a,c,l,mnint)
                tsv=FR_sys4(a,h,c,d,w)

                grafic_FR(tsv,sv3,w,output_folder_MC)
                Espace()

            elif no == 8:
                """8. Poles in open loop."""
                print('Poles in open loop.')
                # c_scaling_matrix
                su_diag,sx_diag,sy_diag=scaling_matrix()
                ap,bp,cp,dp=scaled_dinamics(ap0,bp0,cp0,dp0,su_diag, sx_diag, sy_diag)
                # e_aument_plant
                ns, nc=shapebp(bp)
                a, b, c, d=PA_abcd(ap,bp,cp,nc,ns)
                # f_Kalman_Filter
                ll,lh,l=FK_c(ap,bp,cp,dp)
                pnint,mu,mnint=constants_NI(nc)
                g1,h=care_funtion(a,c,l,mnint)
                # g_poles_open_loop
                poles_OL(a,h,c,nc,w,output_folder_MC)
                Espace()

            elif no == 9:
                """9. LQR control design."""
                print('LQR control design.')
                # c_scaling_matrix
                su_diag,sx_diag,sy_diag=scaling_matrix()
                ap,bp,cp,dp=scaled_dinamics(ap0,bp0,cp0,dp0,su_diag, sx_diag, sy_diag)
                # e_aument_plant
                ns, nc=shapebp(bp)
                a, b, c, d=PA_abcd(ap,bp,cp,nc,ns)
                # f_Kalman_Filter
                ll,lh,l=FK_c(ap,bp,cp,dp)
                pnint,mu,mnint=constants_NI(nc)
                g1,h=care_funtion(a,c,l,mnint)
                # h_compensator
                g=LQR(a,b,c,nc)
                ak,bk,ck,dk=Compensator_analysis(a,b,c,g,h,nc,ns)
                print_controlPA(ak,bk,ck,dk)
                compensador_SV(a,h,g,nc,ak,bk,ck,w,output_folder_MC)
                Espace()

            elif no == 10:
                """10. Open loop analysis."""
                print('Open loop analysis.')
                # c_scaling_matrix
                su_diag,sx_diag,sy_diag=scaling_matrix()
                ap,bp,cp,dp=scaled_dinamics(ap0,bp0,cp0,dp0,su_diag, sx_diag, sy_diag)
                # e_aument_plant
                ns, nc=shapebp(bp)
                a, b, c, d=PA_abcd(ap,bp,cp,nc,ns)
                # f_Kalman_Filter
                ll,lh,l=FK_c(ap,bp,cp,dp)
                pnint,mu,mnint=constants_NI(nc)
                g1,h=care_funtion(a,c,l,mnint)
                tsv=FR_sys4(a,h,c,d,w)
                # h_compensator
                g=LQR(a,b,c,nc)
                ak,bk,ck,dk=Compensator_analysis(a,b,c,g,h,nc,ns)
                # i_OL_CL
                al,bl,cl,dl=open_loop_analysis(ap,bp,cp,ak,bk,ck,nc,ns)
                printOL(al,bl,cl,dl)
                graf_OL(al,bl,cl,w,nc,tsv,output_folder_MC)
                Espace()

            elif no == 11:
                """11. Closed loop analysis."""
                print('Closed loop analysis.')
                # c_scaling_matrix
                su_diag,sx_diag,sy_diag=scaling_matrix()
                ap,bp,cp,dp=scaled_dinamics(ap0,bp0,cp0,dp0,su_diag, sx_diag, sy_diag)
                # e_aument_plant
                ns, nc=shapebp(bp)
                a, b, c, d=PA_abcd(ap,bp,cp,nc,ns)
                # f_Kalman_Filter
                ll,lh,l=FK_c(ap,bp,cp,dp)
                pnint,mu,mnint=constants_NI(nc)
                g1,h=care_funtion(a,c,l,mnint)
                # h_compensator
                g=LQR(a,b,c,nc)
                ak,bk,ck,dk=Compensator_analysis(a,b,c,g,h,nc,ns)
                # i_OL_CL
                al,bl,cl,dl=open_loop_analysis(ap,bp,cp,ak,bk,ck,nc,ns)
                poles_cl_(al,bl,cl,a,b,c,h,g)
                grafic_cl(al,bl,cl,w,nc,output_folder_MC)
                Espace()

            elif no == 12:
                """12. Step response in closed loop. """
                print('Step response in closed loop.')
                # c_scaling_matrix
                su_diag,sx_diag,sy_diag=scaling_matrix()
                ap,bp,cp,dp=scaled_dinamics(ap0,bp0,cp0,dp0,su_diag, sx_diag, sy_diag)
                # e_aument_plant
                ns, nc=shapebp(bp)
                a, b, c, d=PA_abcd(ap,bp,cp,nc,ns)
                # f_Kalman_Filter
                ll,lh,l=FK_c(ap,bp,cp,dp)        
                pnint,mu,mnint=constants_NI(nc)
                g1,h=care_funtion(a,c,l,mnint)
                # h_compensator
                g=LQR(a,b,c,nc)
                ak,bk,ck,dk=Compensator_analysis(a,b,c,g,h,nc,ns)
                # i_OL_CL
                al,bl,cl,dl=open_loop_analysis(ap,bp,cp,ak,bk,ck,nc,ns)
                # j_SR_CL
                grafics_SR_CL(al,bl,cl,nc,output_folder_MC)
                Espace()

            elif no == 13:
                """13. Step response in closed loop (with alternative function "feedback")."""
                print('Step response in closed loop (with alternative function "feedback").')
                # c_scaling_matrix
                su_diag,sx_diag,sy_diag=scaling_matrix()
                ap,bp,cp,dp=scaled_dinamics(ap0,bp0,cp0,dp0,su_diag, sx_diag, sy_diag)
                # e_aument_plant
                ns, nc=shapebp(bp)
                a, b, c, d=PA_abcd(ap,bp,cp,nc,ns)
                # f_Kalman_Filter
                ll,lh,l=FK_c(ap,bp,cp,dp)
                pnint,mu,mnint=constants_NI(nc)
                g1,h=care_funtion(a,c,l,mnint)
                # h_compensator
                g=LQR(a,b,c,nc)
                ak,bk,ck,dk=Compensator_analysis(a,b,c,g,h,nc,ns)
                # k_ACL_SR
                sys8, sys1, Tsys=ACL_System(ak,bk,ck,dk,ap, bp, cp, dp)
                plot_FR_Tsys(Tsys,w,output_folder_MC)
                SR_Tsys(Tsys,output_folder_MC)
                Espace()

            elif no == 14:
                """14. LQG/LTR Controller for the Augmented plant."""
                print('LQG/LTR Controller for the Augmented plant.')
                # c_scaling_matrix
                su_diag,sx_diag,sy_diag=scaling_matrix()
                ap,bp,cp,dp=scaled_dinamics(ap0,bp0,cp0,dp0,su_diag, sx_diag, sy_diag)
                # e_aument_plant
                ns, nc=shapebp(bp)
                a, b, c, d=PA_abcd(ap,bp,cp,nc,ns)
                # f_Kalman_Filter
                ll,lh,l=FK_c(ap,bp,cp,dp)
                pnint,mu,mnint=constants_NI(nc)
                g1,h=care_funtion(a,c,l,mnint)
                # h_compensator
                g=LQR(a,b,c,nc)
                ak,bk,ck,dk=Compensator_analysis(a,b,c,g,h,nc,ns)
                # i_OL_CL
                al,bl,cl,dl=open_loop_analysis(ap,bp,cp,ak,bk,ck,nc,ns)
                # k_ACL_SR
                sys8, sys1, Tsys=ACL_System(ak,bk,ck,dk,ap, bp, cp, dp)
                # l_CS_KS
                U=Def_U(sys8,al,bl,cl,nc)
                SR_U(U,output_folder_MC)
                SC_KS(U,w,output_folder_MC)
                Espace()

            elif no == 15:
                """15.Plot the responses with the actual controller (augmented and scaled). """
                print('Plot the responses with the actual controller (augmented and scaled). ')
                # c_scaling_matrix
                su_diag,sx_diag,sy_diag=scaling_matrix()
                ap,bp,cp,dp=scaled_dinamics(ap0,bp0,cp0,dp0,su_diag, sx_diag, sy_diag)
                # e_aument_plant
                ns, nc=shapebp(bp)
                a, b, c, d=PA_abcd(ap,bp,cp,nc,ns)
                # f_Kalman_Filter
                ll,lh,l=FK_c(ap,bp,cp,dp)
                pnint,mu,mnint=constants_NI(nc)
                g1,h=care_funtion(a,c,l,mnint)
                # h_compensator
                g=LQR(a,b,c,nc)
                ak,bk,ck,dk=Compensator_analysis(a,b,c,g,h,nc,ns)
                # m_CLS
                akr,bkr,ckr,dkr=RCL_System(ak,bk,ck,dk,su_diag,sy_diag,folder_path)
                Tsys=Rsystem_feedback(akr,bkr,ckr,dkr,ap0, bp0, cp0, dp0)
                Rprint_Tsys(Tsys)
                Rplot_FR_Tsys(Tsys,w,output_folder_MC)
                SR_Tsys_1(Tsys,output_folder_MC)
                Espace()
                print('Generate a real control code in state space: real_controller_data')
                Espace()

            elif no == 16:
                """16. Actual open  and closed loop analysis."""
                print('Actual open  and closed loop analysis.')
                # a_ctrl_obs_eigen
                eval, evec = EingV(ap0)
                # c_scaling_matrix
                su_diag,sx_diag,sy_diag=scaling_matrix()
                ap,bp,cp,dp=scaled_dinamics(ap0,bp0,cp0,dp0,su_diag, sx_diag, sy_diag)
                # e_aument_plant
                ns, nc=shapebp(bp)
                a, b, c, d=PA_abcd(ap,bp,cp,nc,ns)
                # f_Kalman_Filter
                ll,lh,l=FK_c(ap,bp,cp,dp)
                pnint,mu,mnint=constants_NI(nc)
                g1,h=care_funtion(a,c,l,mnint)
                tsv=FR_sys4(a,h,c,d,w)
                # h_compensator
                g=LQR(a,b,c,nc)
                ak,bk,ck,dk=Compensator_analysis(a,b,c,g,h,nc,ns)
                # m_CLS
                akr,bkr,ckr,dkr=RCL_System(ak,bk,ck,dk,su_diag,sy_diag,folder_path)
                # n_Real_OL_CL
                alr,blr,clr,dlr=olreal(ap0,bp0,cp0,akr,bkr,ckr,ns,nc)
                print_COL(alr,blr,clr,dlr)
                Zeros_RF_OL(alr,blr,clr,dlr,w,tsv,output_folder_MC)
                
                sysclr,svclr,sysclr1,svclr1,sysclr2=clreal_FR(alr,blr,clr,dlr,w,nc,output_folder_MC)
                plot_FR_CL(sysclr,svclr,sysclr1,svclr1,w,output_folder_MC)
                SR_CL(sysclr1,output_folder_MC)
                t,y= LSIM_CL(sysclr2)
                plot_LSIM_CL(t,y,output_folder_MC)
                Espace()

            elif no == 17:
                """17. Plot the actual control signal Tp1, Tp2, and Tp3. """
                print('Plot the actual control signal Tp1, Tp2, and Tp3.')
                # c_scaling_matrix
                su_diag,sx_diag,sy_diag=scaling_matrix()
                ap,bp,cp,dp=scaled_dinamics(ap0,bp0,cp0,dp0,su_diag, sx_diag, sy_diag)
                # e_aument_plant
                ns, nc=shapebp(bp)
                a, b, c, d=PA_abcd(ap,bp,cp,nc,ns)
                # f_Kalman_Filter
                ll,lh,l=FK_c(ap,bp,cp,dp)
                pnint,mu,mnint=constants_NI(nc)
                g1,h=care_funtion(a,c,l,mnint)
                tsv=FR_sys4(a,h,c,d,w)
                # h_compensator
                g=LQR(a,b,c,nc)
                ak,bk,ck,dk=Compensator_analysis(a,b,c,g,h,nc,ns)
                # m_CLS
                akr,bkr,ckr,dkr=RCL_System(ak,bk,ck,dk,su_diag,sy_diag,folder_path)
                # n_Real_OL_CL
                alr,blr,clr,dlr=olreal(ap0,bp0,cp0,akr,bkr,ckr,ns,nc)
                # o_Real_Control
                Ur=Controll_FR(alr,blr,clr,akr,bkr,ckr,dkr,nc)
                SR_Ur(Ur,output_folder_MC)
                SC_KS_UR(Ur,w,output_folder_MC)
                Espace()

            elif no == 18:
                """18. Obtain the state space matrices of the controller in discrete time."""
                print('Obtain the state space matrices of the controller in discrete time.')
                print('Real control in state space')
                from real_state_space_and_discretized_control.real_controller_data import akr, bkr, ckr, dkr
                # p_discretizado
                Ts = 1
                discretize(akr, bkr, ckr, dkr,Ts,folder_path)
                Espace()
                Ts = 0.1
                discretize(akr, bkr, ckr, dkr,Ts,folder_path)
                Espace()

            else:
                Invalid_option()
                Espace()

        except ValueError:
            Invalid_number()
Cltr_no()
#***************************************************************************
while True:
    print("Do you want to continue with the simulation of the LQG/LTR control operation? ")
    continuar_simulacion = input("yes[y] or no[n]: ")
    Espace()

    if continuar_simulacion == 'y' or continuar_simulacion == 'Y':
        Cltr_no()
    elif continuar_simulacion == 'n' or continuar_simulacion == 'N':
        print("The process has been completed.")
        break  
    else:
        Invalid_option()
        Espace()
