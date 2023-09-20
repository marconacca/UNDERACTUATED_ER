import numpy as np
import pinocchio as pin
from energy_shaping import energy_shaping_controller
from stabilization import stabilization_controller
from energy_analysis import compute_energy
from energy_analysis import compute_energy2002

bool = False

# Implement the swing-up control algorithm
def swing_up_control(q, qdot, initial_state, desired_state):
    global bool
    
    #threshold for LQR activation
    eps = 0.04        # (0.04 2007 paper) in state subtraction absolute values
    
    actual_state = np.concatenate((q, qdot))
    state_error = actual_state - desired_state
    
    
    # **********   ENERGY CONTROLLER   **********
    #total_energy, desired_energy, M, C, G, M_det, gains = compute_energy2002(q, qdot)
    total_energy, desired_energy, M, C, G, M_det, gains = compute_energy( q, qdot)  
    control_torques_energy_shaping, energy_error = energy_shaping_controller( total_energy, desired_energy, q, qdot, M, C, G, M_det, gains)

    # **********   STABILIZATION LQR CONTROLLER   **********
    control_torques_stabilization = stabilization_controller(q, qdot, state_error)



    # **********   THRESHOLD CHECK to switch from Energy to LQR Controller   **********
    
    condition = abs(state_error[0]) + abs(state_error[1]) + abs(state_error[2])*0.1 + abs(state_error[3])*0.1
    
    # print("actual_state", actual_state)
    # print("desired_state", desired_state)
    #print('€€€€€ - the State Error vector is: ',state_error)
    #print('€€€€€  the State Condition is: ', condition)
    print("@@@@@@  condition: ", condition)

    
    #Combine control torques
    if condition < eps or bool == True:
        control_torques = control_torques_stabilization
        print("-------------------------------LQR-CONTROL:    press ENTER to continue:")
        bool = True
    else:
        control_torques = control_torques_energy_shaping



    return control_torques, energy_error