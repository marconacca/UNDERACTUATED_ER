def energy_shaping_controller(potential_energy, desired_energy):
    # Define controller gains
    kp = 1.0  # Proportional gain
    kd = 0.5  # Derivative gain

    # Compute the error between desired energy and current energy
    error = desired_energy - potential_energy

    # Compute control torques
    control_torques = -kp * error

    return control_torques