import numpy as np
import sim_utils
import pybullet as pb
from acro_dynamics import *




def simulate():
    # Set the desired state for stabilization
    desired_state = np.array([1.4, 0, 0, 0])  # [q1, q2, dq1, dq2]

    # Define the control loop parameters
    dt = 1/240  # Time step
    duration = 25.0  # Duration of the control loop
    num_steps = int(duration / dt)  # Number of control steps

    # Initialize the current state
    current_state = np.array([1.57, 0, 0, 0])  # [q1, q2, dq1, dq2]

    # Initialize a list to store the states over time
    state_history = [current_state]
    jointIndices = np.array([0,1])
    robotID, robotModel = sim_utils.simulationSetup(num_steps)
    for i in jointIndices:
        pb.resetJointState(robotID, i, current_state[i])
        dot_current_state = np.array([0,0])
    # Control loop
    input("press ENTER to START the simulation:")
    for _ in range(num_steps):
    
        # Compute the control input based on the current state
        error = desired_state - current_state
        print("STO STAMPANDO L'ERRORE: \n")
        print(error)
        K = np.array([-460.5540, -171.0658,  -69.2076,  -26.9682])
        control_input = np.dot(-K, error)  # Apply control law, K is the controller gain matrix
        print("STO STAMPANDO IL TORQUE: \n")
        print(control_input)
        control_torque = np.array([0,control_input])
        if np.isnan(control_torque[1]):
            break
        pb.setJointMotorControlArray(robotID, jointIndices, controlMode = pb.TORQUE_CONTROL, forces=control_torque)
        # Simulate the dynamics of the acrobot for one time step
        next_state = acrobot_dynamics(current_state[0], current_state[1], dot_current_state[0], dot_current_state[1], control_input, dt)  # Update the state based on dynamics

        current_state[0] = next_state[0]
        current_state[1] = next_state[1]
        dot_current_state[0] = next_state[2]
        dot_current_state[1] = next_state[3]
        
        current_state[0] = current_state[0] % (2*np.pi)
        current_state[1] =  current_state[1] % (2*np.pi)
        # Update the current state
        #current_state = next_state

        # Store the current state in the history list
        state_history.append(current_state)



    # Convert the state history list to a NumPy array
    state_history = np.array(state_history)
    
    

    input("press ENTER to CLOSE the simulation:")

    pb.disconnect()