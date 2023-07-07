import numpy as np

def stabilization_controller(q, qdot, desired_position, desired_velocity):
    # Define controller gains
    kp = 1.0  # Proportional gain
    kd = 0.5  # Derivative gain


    # we should use a LQR control to stabilize at the end the acrobot (putting a threshold of the error in position)

    # Compute the position error (the desired position is represent by the [x, y] desired angle? i take just 1 angle)
    #desired_position[0] = - np.pi/2
    position_error = desired_position - q
    #print(' desired_position = ', desired_position)
    print(' pos_err = ', position_error)

    # Compute the velocity error
    velocity_error = desired_velocity - qdot

    state_error = np.concatenate((position_error,velocity_error))
    actual_state = np.concatenate((q,qdot))


    # Matrice K of LQR controller
    K = np.array([-246.481, -98.690, -106.464, -50.138])
    #K = np.array([-2.481, -0.9690, -0.1464, -0.5138])


    # Compute control torques
    tau2 = np.dot(-K, state_error)
    #tau2 = np.dot(-K, actual_state)

    control_torques = np.array([0, tau2])


    print('actual_state = ', actual_state)
    print('tau2 = ', tau2)



    return state_error, control_torques