def stabilization_controller(q, qdot, desired_position):
    # Define controller gains
    kp = 1.0  # Proportional gain
    kd = 0.5  # Derivative gain


    # we should use a LQR control to stabilize at the end the acrobot (putting a threshold of the error in position)

    # Compute the position error (the desired position is represent by the [x, y] desired angle? i take just 1 angle)
    position_error = desired_position[0] - q[0]
    #print(' desired_position = ', desired_position)
    #print(' q0 = ', q[0])
    print(' pos_err = ', position_error)

    # Compute the velocity error
    velocity_error = -qdot[0]

    # Compute control torques
    control_torques = -kp * position_error - kd * velocity_error

    return position_error, control_torques