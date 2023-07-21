import numpy as np
import math
import pinocchio as pin

def energy_shaping_controller(robot, current_energy, desired_energy, q, qdot, M, C, G, M_det, gains):

    '''
    The current energy is the current (at a time instant) total energy of the system
    The desired energy is the total energy when the acrobot is at rest in vertical position (so just potential energy in theory)

    '''


# #########################   defining Parameters for the Control Law   #########################

    # dynamics and control components
    #M_det = np.linalg.det(M)
    #coriolis_forces = np.dot(C, qdot)
    h1 = C[0]
    h2 = C[1]


    kp = gains[0]
    kd = gains[1]
    kv = gains[2]
    ke = gains[3]

    # Compute the error between desired energy and current energy
    energy_error = current_energy - desired_energy


    # #########################   CONTROL LAW tau2   #########################

    # Compute control torques
    #nom = - ((kv*qdot[1] + kp*q[1])*M_det + kd*(M[1,0]*(h1+G[0])-M[0,0]*(h2+G[1])))
    #den = kd*M[0,0] + energy_error*M_det
    #tau2 = nom / den
    #tau2 = -((kv*qdot[1] + kp*q[1])*M_det + kd*(M[1,0]*(h1+G[0])-M[0,0]*(h2+G[1])))  / ( energy_error*M_det+kd*M[0,0] )

    # Torque OLD PAPER 2002
    nom = - ((kv*qdot[1] + kp*q[1])*M_det + kd*(M[1,0]*(h1+G[0])-M[0,0]*(h2+G[1])))
    den =  ke*energy_error*M_det + kd*M[0,0]
    tau2 = nom / den
    
    
    control_torques = np.array([0, tau2])

    # #########################  --------------------  #########################



    # #########################   Values Check   #########################

    #print('\nq configuration is: ', q)
    #print('qdot joint velocity is: ', qdot)
    print('\nM is: ', M)
    print('C is: ', C)
    print('G is: ', G)
    print('M_det is: ', M_det)
    print("\nCurrent energy: ", current_energy)
    print("Desired energy: ", desired_energy)
    print('€€€€€ - Energy Error is: ', energy_error)
    #print('  Tau2 is: ', tau2)

    return control_torques, energy_error