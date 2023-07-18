import numpy as np
import pinocchio as pin
from energy_shaping import energy_shaping_controller
from stabilization import stabilization_controller
from energy_analysis import compute_energy

# Implement the swing-up control algorithm using Pinocchio
def swing_up_control(model, q, qdot):

    # Energy shaping control
    total_energy, desired_energy, M, C, G, gains = compute_energy(model, q, qdot)
    control_torques_energy_shaping, energy_error = energy_shaping_controller(model, total_energy, desired_energy, q, qdot, M, C, G, gains)

    #current energy = current total energy of the system
    #desired energy = total energy when the acrobot is at rest in vertical position (ideally only potential energy)

    return control_torques_energy_shaping, energy_error

def stabilization_control(model, q, qdot, desired_position, desired_velocity):

    # Stabilization control (LQR)
    control_torques_stabilization, state_error = stabilization_controller(q, qdot, desired_position, desired_velocity)

    return control_torques_stabilization, state_error

def switch(q, qdot):

    zeta = 0.04 #threshold to switch controller (Energy to LQR) taken from paper
    x_state = np.array([q[0]-np.pi/2, q[1], qdot[0], qdot[1]])
    if ((abs(x_state[0]) + abs(x_state[1]) + 0.1*(abs(x_state[2])) + 0.1*(abs(x_state[3]))) < zeta):
        return True
    
    return False