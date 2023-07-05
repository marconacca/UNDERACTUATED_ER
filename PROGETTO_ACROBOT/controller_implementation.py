import pinocchio as pin
from energy_shaping import energy_shaping_controller
from stabilization import stabilization_controller
from energy_analysis import compute_energy

# Implement the swing-up control algorithm using Pinocchio
def swing_up_control(model, q, qdot, desired_position):
    # Energy shaping control
    total_energy, desired_energy, M, C, G, gains = compute_energy(model, q, qdot)
    control_torques_energy_shaping = energy_shaping_controller(model, total_energy, desired_energy, q, qdot, M, C, G, gains)

    # Stabilization control
    #pos_err, control_torques_stabilization = stabilization_controller(q, qdot, desired_position)

    #threshold to change the controller from energy to LQR
    eps = 0.3   # in rad or xz-position?

    # Combine control torques
    # if abs(pos_err) < eps:
    #     control_torques = control_torques_energy_shaping
    # else:
    #     control_torques = control_torques_stabilization
    control_torques = control_torques_energy_shaping

    #tau = pin.rnea(model, q, v, control_torques)

    return control_torques