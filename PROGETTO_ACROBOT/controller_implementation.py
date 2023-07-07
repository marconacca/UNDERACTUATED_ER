import numpy as np
import pinocchio as pin
from energy_shaping import energy_shaping_controller
from stabilization import stabilization_controller
from energy_analysis import compute_energy

# Implement the swing-up control algorithm using Pinocchio
def swing_up_control(model, q, qdot, desired_position, desired_velocity):
    # Energy shaping control
    total_energy, desired_energy, M, C, G, gains = compute_energy(model, q, qdot)
    control_torques_energy_shaping = energy_shaping_controller(model, total_energy, desired_energy, q, qdot, M, C, G, gains)

    # Stabilization control
    state_error, control_torques_stabilization = stabilization_controller(q, qdot, desired_position, desired_velocity)

    #threshold to change the controller from energy to LQR
    eps = 3.0       # in rad or xz-position?

    # Combine control torques
    # if (abs(state_error[0]) < eps) and (abs(state_error[1]) < eps):
    #     control_torques = control_torques_stabilization
    #     input("LQR-CONTROL:    press ENTER to continue:")
    # else:
    #     control_torques = control_torques_energy_shaping
    #     #input("ENERGY-CONTROL:    press ENTER to continue:")

    #control_torques = control_torques_energy_shaping
    control_torques = control_torques_stabilization

    #tau = pin.rnea(model, q, v, control_torques)

    print('€€€€€ the state error vector is: ',state_error)

    return control_torques