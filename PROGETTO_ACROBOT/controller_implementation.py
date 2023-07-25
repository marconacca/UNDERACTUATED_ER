import numpy as np
import pinocchio as pin
from energy_shaping import energy_shaping_controller
from stabilization import stabilization_controller
from energy_analysis import compute_energy22

# Implement the swing-up control algorithm using Pinocchio
def swing_up_control(model, q, qdot, desired_position, desired_velocity):
    # Energy shaping control
    #total_energy, desired_energy, M, C, G, M_det, gains = compute_energy(model, q, qdot)
    

    

    #threshold to change the controller from energy to LQR
    eps = 0.04       # in rad or xz-position?
    state_paper = np.array([q[0]-np.pi/2, q[1], qdot[0], qdot[1]])
    condition = abs(state_paper[0]) + abs(state_paper[1]) + abs(state_paper[2])*0.1 + abs(state_paper[3])*0.1


    #print('€€€€€ - the State Error vector is: ',state_error)
    print('€€€€€  the State Condition is: ', condition)

    #Combine control torques
    #if (abs(state_error[0]) < eps) and (abs(state_error[1]) < 3*eps):
    if condition < eps:
        # Stabilization control
        control_torques_stabilization, state_error = stabilization_controller(q, qdot, desired_position, desired_velocity)
        control_torques = control_torques_stabilization
        energy_error = state_error
        input("LQR-CONTROL:    press ENTER to continue:")
    else:
        total_energy, desired_energy, M, C, G, M_det, gains = compute_energy22(model, q, qdot)  
        control_torques_energy_shaping, energy_error = energy_shaping_controller(model, total_energy, desired_energy, q, qdot, M, C, G, M_det, gains)
        control_torques = control_torques_energy_shaping
        #input("ENERGY-CONTROL:    press ENTER to continue:")

    #control_torques = control_torques_energy_shaping
    #control_torques = control_torques_stabilization

    #tau = pin.rnea(model, q, v, control_torques)

    return control_torques, energy_error