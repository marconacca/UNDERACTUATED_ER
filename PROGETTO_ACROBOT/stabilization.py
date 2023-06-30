def stabilization_controller(q, v, desired_position):
    # Define controller gains
    kp = 1.0  # Proportional gain
    kd = 0.5  # Derivative gain

    # Compute the position error
    position_error = desired_position - q[0]

    # Compute the velocity error
    velocity_error = -v[0]

    # Compute control torques
    control_torques = -kp * position_error - kd * velocity_error

    return control_torques