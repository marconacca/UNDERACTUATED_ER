import numpy as np

def stabilization_controller(q, qdot, desired_position, desired_velocity):
    # we should use a LQR control to stabilize at the end the acrobot (putting a threshold of the error in position)

    # Compute the position error
    position_error = q - desired_position

    # Compute the velocity error
    velocity_error = qdot - desired_velocity

    state_error = np.concatenate((position_error,velocity_error))

    x = np.array([q[0]-np.pi/2, q[1], qdot[0], qdot[1]])

    # Matrice K of LQR controller of the PAPER parameters
    K = np.array([-246.481, -98.690, -106.464, -50.138])

    # Matrice K for our model parameters
    #K = np.array([-460.5540, -171.0658,  -69.2076,  -26.9682])

    # Compute control torques
    tau2 = np.dot(-K, x)

    control_torques = np.array([0, tau2])

    return control_torques, state_error