import numpy as np
import math
import pinocchio as pin

def energy_shaping_controller(robot, current_energy, desired_energy, q, qdot, M, C, G, gains):

    '''
    The current energy is the current (at a time instant) total energy of the system
    The desired energy is the total energy when the acrobot is at rest in vertical position (so just potential energy in theory)

    '''


# #########################   defining Parameters for the Control Law   #########################

    # dynamics and control components
    M_det = np.linalg.det(M)
    #coriolis_forces = np.dot(C, qdot)
    h1 = C[0]
    h2 = C[1]


    kp = gains[0]
    kd = gains[1]
    kv = gains[2]

    # Compute the error between desired energy and current energy
    energy_error = current_energy - desired_energy



    # #########################   CONTROL LAW tau2   #########################

    # Compute control torques
    tau2 = -( (kv*qdot[1] + kp*q[1])*M_det + kd*(M[1,0]*(h1+G[0])-M[0,0]*(h2+G[1])) ) / ( energy_error*M_det+kd*M[0,0] )

    control_torques = np.array([0, tau2])

    # #########################  --------------------  #########################



    # #########################   Values Check   #########################

    return control_torques