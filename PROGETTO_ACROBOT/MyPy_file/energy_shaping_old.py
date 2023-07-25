import numpy as np
import math
import pinocchio as pin

def energy_shaping_controller(robot, total_energy, desired_energy, alphas, betas, q, qdot, M, C, G, gains):

    '''
    The total energy is the total energy of the system
    The desired energy is the total energy when the acrobot is at rest in vertical position (ideally just potential energy)

    '''

    # dynamics and control parameters
    delta = np.linalg.det(M)
    h1 = C[0]
    h2 = C[1]


    kp = gains[0]
    kd = gains[1]
    kv = gains[2]
    ke = gains[3]

    # Compute the error between desired energy and current energy
    energy_error = total_energy - desired_energy

    # Compute control torques
    tau2 = -((kv*qdot[1] + kp*q[1])*delta + kd*(M[1,0]*(h1+G[0])-M[0,0]*(h2+G[1]))) / ((kd*M[0,0]) + (ke*energy_error*delta))

    # Compute bound on the control input, which is free of the control parameters and any initial state of the Acrobot
    tau2_max = (alphas[1]*betas[0] + alphas[2]*betas[1] - alphas[0]*betas[1] - alphas[2]*betas[1]) / (alphas[0] + alphas[1] + alphas[2])
    print("Bound on torque: ", tau2_max)
    
    # Calculate control input u based on energy control law with damping
    #u = total_energy - 0.1 * desired_energy  # You can adjust the damping term (0.1 in this case) as needed

    # Energy error
    #energy_error = total_energy - desired_energy

    # PID control law for torque
    #u = kd * V_dot + kp * energy_error + kv * V_dot

    control_torques = np.array([0, tau2])

    return control_torques, energy_error