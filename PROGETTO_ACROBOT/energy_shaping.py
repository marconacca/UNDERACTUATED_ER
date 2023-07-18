import numpy as np
import math
import pinocchio as pin

def energy_shaping_controller(robot, current_energy, desired_energy, q, qdot, M, C, G, gains):

    '''
    The current energy is the current (at a time instant) total energy of the system
    The desired energy is the total energy when the acrobot is at rest in vertical position (so just potential energy in theory)

    '''

    # dynamics and control parameters
    delta = np.linalg.det(M)
    h1 = C[0]
    h2 = C[1]


    kp = gains[0]
    kd = gains[1]
    kv = gains[2]

    # Compute the error between desired energy and current energy
    energy_error = current_energy - desired_energy

    # Compute control torques
    tau2 = -((kv*qdot[1] + kp*q[1])*delta + kd*(M[1,0]*(h1+G[0])-M[0,0]*(h2+G[1]))) / ((kd*M[0,0]) + (energy_error*delta))

    control_torques = np.array([0, tau2])

    return control_torques, energy_error