import numpy as np
import math 

def energy_shaping_controller(current_energy, desired_energy, q, qdot, M, C, G):

    '''
    The current energy is the current (at a time instant) total energy of the system
    The desired energy is the total energy when the acrobot is at rest in vertical position (so just potential energy in theory)

    '''

    '''
# -------------------------   Function (to create) to calculate the gain condition and so the possible gain for the control law of the system   -------------------------

    #theta1 = m1*lc1^2 + m2*l1^2 + I1
    #theta2 = m2*lc2^2 + I2
    #theta3 = m2*l1*lc2
    #theta4 = m1*lc1 + m2*l1
    #theta5 = m2*lc2

    #rho_q2 = m[0,0] / M_det  , should be equal to = ( theta1+theta2+2*theta3*cos(q2) ) / (theta1*theta2 - theta3^2*cos(q2)^2 )
    #rho_star = min(rho_q2)   , with all possible q2

    #alpha0 = ( 2*theta3*theta4 +theta1*theta5 ) / (3*theta3*theta5)
    #alpha1 = 
    #alpha2 = 
    #alpha3 = 

    # def switch(alpha0):
    # if alpha0 > 1:
    #     return alpha1^2 + alpha2^2
    # elif alpha0 = 1:
    #     return alpha2
    # elif alpha0 < 1:
    #     return alpha3

    #alpha =

    #beta = theta4/theta5  (has to be > 0 to define eta_q2 or eps_q2)
    #delta_q2 = math.sqrt(1+beta^2+2*beta*cos(q2))

    #eta_q2 = ( (delta_q2 - beta - 1)*sin(q2) ) / (delta_q2*q2)
    #eta_star = max(eta_q2) ,  q2 belong to [pi, 3/2*pi] depends on our reference
    #eps_q2 = ( (delta_q2 + beta + 1)*sin(q2) ) / (delta_q2*q2)
    #eps_star = max(eps_q2) ,  q2 belong to [0, 1/2*pi]  depends on our reference


    # gains condition until theorem 3 (to converges)
    # kd > ke*desired_energy/rho_star
    # kp > max(eta_star,eps_star)*ke*theta4*theta5*g^2    , g is the gravity,  max() is eta_star in this case

    # gains condition until theorem 3 (to converges)
    # kd > ke*desired_energy/rho_star
    # kp > max(eta_star,eps_star)*ke*theta4*theta5*g^2    , g is the gravity,  max() is eps_star in this case
    

    ''' # -------------------------   end function   -------------------------



    # Define controller gains   (this choice is the one of the paper simulation realted to theorem 3 and satisfied theorem 2 condition (31) alpha<>0)
    # kp = 22    # Proportional gain
    # kd = 15    # Derivative gain or inertia-coriolis gain
    # ke = 0.5    # Energy error gain
    # kv = 45    # velocity gain or derivative gain

    kp = 80    # Proportional gain
    kd = 1.0   # Derivative gain or inertia-coriolis gain
    ke = 0.5    # Energy error gain
    kv = 135    # velocity gain or derivative gain

    # dynamics and control components
    M_det = np.linalg.det(M)
    coriolis_forces = np.dot(C, qdot)
    h1 = coriolis_forces[0]
    h2 = coriolis_forces[1]


    # Compute the error between desired energy and current energy
    energy_error = desired_energy - current_energy
    #print('desired_energy is: ', desired_energy)
    print('energy_error is: ', energy_error)

    print('q conf is: ', q)
    print('qdot joint velocity is: ', qdot)


    # Compute control torques
    tau2 = ( -(kv*qdot[1] + kp*q[1])*M_det - kd*(M[1,0]*(h1+G[0])-M[0,0]*(h2+G[1])) ) / ( ke*energy_error*M_det+kd*M[0,0] )
    #tau2 = ke * energy_error
    # print('M is: ', M[1,0])
    # print('C1 is: ', h1, 'and C2 is: ', h2)
    # print('G is: ', G[1])
    # print('q2 is: ', q[1])
    # print('qdot2 is: ', qdot[1])
    print('  Tau2 is: ', tau2)

    control_torques = np.array([0, tau2])

    return control_torques