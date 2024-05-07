def Espace():
     print('---------------------------------------------------------------------------------------')
     print('***************************************************************************************')

def printInitialInfoNL():
     print('The AUV Model Simulator')
     print('***************************************************************************************')
     print('MODEL  NON LINEAR')
   
def InfoTp_initial():
     print('The distances between the thrusters will be observed first.')
     print('The position of the thrusters p1 and p2 are on the AUV bow')
     print('The position of the thruster p3 is at a distance dp3 with respect to the center of gravity and in the aft direction')
     print('---------------------------------------------------------------------------------------')
     print('***************************************************************************************')
     print('The default distance between thrusters p1 and p2 are:                    dp = 0.25 m')
     print('Where dp1=dp2=dp')
     print('Default distance of the thruster p3 and the center of gravity is:        dp3= 0.265 m')
 
def print_dp(dp1,dp2,dp3):
    print('Final values of dp1, dp2, dp3:  ')
    print('dp1 = ',dp1,'m')
    print('dp2 = ',dp2,'m')
    print('dp3 = ',dp3,'m')
    Espace()

def Info_Input_Tp():
     print('The nonlinear model input constants (Tp1, Tp2, Tp3) are the forces provided by the thrusters (p1, p2, p3) respectively')
     print('Place the values of Tp1, Tp2, Tp3 that are in the range of 0-30 N')

def Invalid_option():
     print("Invalid option")

def Invalid_number():
     Espace()
     print("Error: Please enter a valid number.")
     Espace()

def printInitialInfo():
     print('---------------------------------------------------------------------------------------')
     print('***************************************************************************************')
     print('Design of the quadratic optimal regulatory system')
     print('---------------------------------------------------------------------------------------')
     print('***************************************************************************************')

def printInfoCalculosCtrl():
     print('Select the operation regarding to LQG/LTR control.')
     print('1. Controllability and observability of the LTI system.')
     print('2. LTI system.')
     print('3. Frequency response of the LTI system.')
     print('4. Scaling of the LTI system.')
     print('5. Frequency response of the scaled LTI system.')
     print('6. Augmented plant (by integrator).')
     print('7. Kalman filter.')
     print('8. Poles in open loop.')
     print('9. LQR control design.')
     print('10. Open loop analysis.')
     print('11. Closed loop analysis.')
     print('12. Step response in closed loop.')
     print('13. Step response in closed loop (with alternative function "feedback").')
     print('14. LQG/LTR Controller for the Augmented plant.')
     print('15. Plot the responses with the actual controller (augmented and scaled).')
     print('16. Actual open  and closed loop analysis.')
     print('17. Plot the actual control signal Tp1, Tp2, and Tp3.')
     print('18. Obtain the state space matrices of the controller in discrete time.')


