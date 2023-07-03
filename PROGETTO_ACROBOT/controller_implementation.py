import pinocchio as pin
from energy_shaping import energy_shaping_controller
from stabilization import stabilization_controller
from energy_analysis import compute_energy

# Implement the swing-up control algorithm using Pinocchio
def swing_up_control(model, q, v, desired_energy, desired_position, data, robotID):
    # Energy shaping control
    potential_energy, _ = compute_energy(model, q, v)
    #control_torques_energy_shaping = energy_shaping_controller(potential_energy, desired_energy)

    # Stabilization control
    control_torques_stabilization = stabilization_controller(model,q[0],q[1],v[0],v[1],data,robotID,desired_position)

    # Combine control torques
    #control_torques = control_torques_energy_shaping + control_torques_stabilization
    control_torques = control_torques_stabilization
    #tau = pin.rnea(model, q, v, control_torques)

    return control_torques