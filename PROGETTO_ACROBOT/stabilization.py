import numpy as np

def stabilization_controller(q, qdot, desired_position, desired_velocity):
    # we should use a LQR control to stabilize at the end the acrobot (putting a threshold of the error in position)

    # Compute the position error
    position_error = q - desired_position
    #print(' desired_position = ', desired_position)
    #print(' pos_err = ', position_error)

    # Compute the velocity error
    velocity_error = qdot - desired_velocity

    state_error = np.concatenate((position_error,velocity_error))
    actual_state = np.concatenate((q,qdot))


    # Matrice K of LQR controller of the PAPER parameters
    #K = np.array([-246.481, -98.690, -106.464, -50.138])
    #K = np.array([-0.2481, -0.09690, -0.01464, -0.05138])

    # matrice K for our model parameters
    K = np.array([-460.5540, -171.0658,  -69.2076,  -26.9682])


    # Compute control torques
    tau2 = np.dot(-K, state_error)
    #tau2 = np.dot(-K, actual_state)

    control_torques = np.array([0, tau2])


    # print('actual_state = ', actual_state)
    # print('tau2 = ', tau2)



    return state_error, control_torques