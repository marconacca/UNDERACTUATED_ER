import numpy as np
import control
import time
import sim_utils
import pybullet as pb
from acro_dynamics import *
from controller_implementation import swing_up_control

def calculate_control(q1, q2, dq1, dq2):
    # Define the control gains
    Kp = 0.07  # Proportional gain
    Kd = 0.0045  # Derivative gain

    # Define the desired energy profile
    desired_energy = 8.0
    q = np.array([q1,q2])
    qdot = np.array([dq1,dq2])
    # Define the acrobot parameters
    l1 = 0.1425
    l2 = 0.2305
    lc1 = 0.035
    lc2 = 0.1 + l1
    m1 = 0.26703
    m2 = 0.33238
    # inertia for each links (other values are smaller than 10^-8, I keep until 10^-5)
    I1 = 0.000036421
    I2 = 0.000014553
    g = 9.81
    # Compute the total energy of the system
    kinetic_energy = 0.5 * (I1 * dq1**2 + I2 * dq2**2 + m2 * l1**2 * dq1**2 + m2 * (l1 * dq1 + l1 * dq2 * np.cos(q2))**2)
    potential_energy = m2 * g * (l1 * np.cos(q1) + lc2 * np.cos(q1 + q2))
    total_energy = kinetic_energy + potential_energy

    # Compute the control input (torque) based on energy shaping
    energy_error = total_energy - desired_energy
    control_input = -Kp * energy_error - Kd * (dq1 + dq2) * kinetic_energy
    return control_input

def simulate():
    simDT = 1/240 # simulation timestep   (was 1/240)
    simTime = 25 # total simulation time in seconds (was 25)
    # Set the desired state for stabilization
    desired_state = np.array([np.pi/2, 0])  # [q1, q2, dq1, dq2]
    dot_desired_state = np.array([0, 0])
    # Define the control loop parameters

    # Initialize the current state
    current_state = np.array([0, 0])  # [q1, q2, dq1, dq2]
    dot_current_state = np.array([0, 0])
    
    jointIndices = np.array([0,1])
    robotID, robotModel = sim_utils.simulationSetup(simDT)
    for i in jointIndices:
        pb.resetJointState(robotID, i, current_state[i])
    # Control loop
    input("press ENTER to START the simulation:")
    for _ in range(int(simTime/simDT)):
        # Compute the control input based on the current state
        error = desired_state - current_state
        print("STO STAMPANDO L'ERRORE: \n")
        print(error)
        if error[0] < 0.2 and error[1] < 0.3:
            break
        control_input =  calculate_control(current_state[0], current_state[1], dot_current_state[0], dot_current_state[1])# Apply control law, K is the controller gain matrix
        print("STO STAMPANDO IL TORQUE: \n")
        print(control_input)
        control_torque = np.array([0,control_input])
        pb.setJointMotorControlArray(robotID, jointIndices, controlMode = pb.TORQUE_CONTROL, forces=control_torque)
        # Simulate the dynamics of the acrobot for one time step
        # next_state = acrobot_dynamics(current_state, control_input[1], dt)  # Update the state based on dynamics
        current_state, dot_current_state = sim_utils.getState(robotID, jointIndices)
        pb.stepSimulation()
        time.sleep(simDT)
    # Convert the state history list to a NumPy array
    state_history = np.array(state_history)

    # Plot the results or perform further analysis
    # ...

    input("press ENTER to CLOSE the simulation:")

    pb.disconnect()