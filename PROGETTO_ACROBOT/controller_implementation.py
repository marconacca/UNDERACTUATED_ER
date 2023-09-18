import numpy as np
import pinocchio as pin
from energy_shaping import energy_shaping_controller
from stabilization import stabilization_controller
from energy_analysis import compute_energy
from energy_analysis import compute_energy22

# Implement the swing-up control algorithm
def swing_up_control(q, qdot, initial_state, desired_state):
    
    # **********   Energy shaping control   **********
    #total_energy, desired_energy, M, C, G, M_det, gains = compute_energy(model, q, qdot)
    total_energy, desired_energy, M, C, G, M_det, gains = compute_energy( q, qdot)  
    control_torques_energy_shaping, energy_error = energy_shaping_controller( total_energy, desired_energy, q, qdot, M, C, G, M_det, gains)

    # **********   Stabilization control   **********
    state_error, control_torques_stabilization = stabilization_controller(q, qdot, desired_state[:2], desired_state[2:])


    # **********   THRESHOLD CHECK to change from Energy to LQR Controller   **********
    eps = 0.08        # (0.04 2007 paper) in state subtraction absolute values
    state_condition = initial_state - desired_state
    condition = abs(state_condition[0]) + abs(state_condition[1]) + abs(state_condition[2])*0.1 + abs(state_condition[3])*0.1
    print("@@@@@@  condition subtraction: ", condition)

    #print('€€€€€ - the State Error vector is: ',state_error)
    #print('€€€€€  the State Condition is: ', condition)

    #Combine control torques
    if condition < eps:
        control_torques = control_torques_stabilization
        input("LQR-CONTROL:    press ENTER to continue:")
    else:
        control_torques = control_torques_energy_shaping
        #input("ENERGY-CONTROL:    press ENTER to continue:")


    #control_torques = control_torques_energy_shaping
    #control_torques = control_torques_stabilization

    # Pinocchio control computation
    #tau = pin.rnea(model, q, v, control_torques)

    return control_torques, energy_error