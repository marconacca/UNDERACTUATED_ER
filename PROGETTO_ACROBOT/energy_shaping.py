import numpy as np
import math
import pinocchio as pin

def energy_shaping_controller(robot, current_energy, desired_energy, q, qdot, M, C, G, gains):

    '''
    The current energy is the current (at a time instant) total energy of the system
    The desired energy is the total energy when the acrobot is at rest in vertical position (so just potential energy in theory)

    '''

    #pin.forwardKinematics(robot.model, robot.data, q)

    # Get the link lengths
    # for i in range(1, robot.model.njoints):  # Skip the base link
    #     parent_joint_id = robot.model.parents[i]
    #     link_placement = robot.data.oMi[i]
    #     parent_link_placement = robot.data.oMi[parent_joint_id]
    #     link_length = np.linalg.norm((parent_link_placement.translation - link_placement.translation))
    #     print(f"Link {i} length: {link_length}")



    # dynamics and control components
    M_det = np.linalg.det(M)
    #coriolis_forces = np.dot(C, qdot)
    h1 = C[0]
    h2 = C[1]


    kp = gains[0]
    kd = gains[1]
    kv = gains[2]



    


    # Compute the error between desired energy and current energy
    energy_error = desired_energy - current_energy
    #print('desired_energy is: ', desired_energy)
    print('energy_error is: ', energy_error)

    print('q conf is: ', q)
    print('qdot joint velocity is: ', qdot)


    # Compute control torques
    tau2 = -( (kv*qdot[1] + kp*q[1])*M_det + kd*(M[1,0]*(h1+G[0])-M[0,0]*(h2+G[1])) ) / ( energy_error*M_det+kd*M[0,0] )
    #tau2 = ke * energy_error
    print('M is: ', M)
    print('C is: ', C)
    print('G is: ', G)
    print('[q1,q2] is: ', q)
    print('qdot is: ', qdot)
    print('  Tau2 is: ', tau2)

    control_torques = np.array([0, tau2])

    return control_torques