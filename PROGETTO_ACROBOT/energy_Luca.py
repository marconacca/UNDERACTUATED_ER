import numpy as np
import time
import sim_utils
import pybullet as pb
import plot_utils
import matplotlib.pyplot as plt
import shutil
from acro_dynamics import *
from controller_implementation import swing_up_control
import os

energy = 0

def calculate_control(q1, q2, dq1, dq2):
    # Define the control gains
    # kp = 0.2    # Proportional gain    > 0.1970336
    # kd = 0.015   # Dynamics gain       > 0.0.0143357
    # kv = 0.21
    kp = 61.2    # Proportional gain
    kd = 35.8    # Dynamics gain
    kv = 1    # Derivative gain

    # Define the desired energy profile
    q = np.array([q1,q2])
    qdot = np.array([dq1,dq2])
    # Define the acrobot parameters
    
    # l1 = 0.1425
    # l2 = 0.2305
    # lc1 = 0.035
    # lc2 = 0.1 + l1
    # m1 = 0.26703
    # m2 = 0.33238
    # # inertia for each links (other values are smaller than 10^-8, I keep until 10^-5)
    # I1 = 0.000036421
    # I2 = 0.000014553

    l1 = 1
    l2 = 2
    lc1 = 0.5
    lc2 = 1
    m1 = 1
    m2 = 1
    # inertia for each links (other values are smaller than 10^-8, I keep until 10^-5)
    I1 = 0.083
    I2 = 0.33

    g = 9.81
    # Compute the total energy of the system
    alpha1 = m1*(lc1**2) + m2*(l1**2) + I1
    alpha2 = m2*(lc2**2) + I2
    alpha3 = m2*l1*lc2
    beta1 = (m1*lc1 + m2*l1)*g
    beta2 = m2*lc2*g

    M = np.array([[alpha1 + alpha2 + 2*alpha3*np.cos(q2), alpha2 + alpha3*np.cos(q2)], [alpha2 + alpha3*np.cos(q2), alpha2]])
    M21 = alpha2 + alpha3*np.cos(q2)
    M11 = alpha1 + alpha2 + 2*alpha3*np.cos(q2)
    BA = np.array([0, 1]).T
    H = alpha3*np.sin(q[1] - q[0])*(np.array([-(qdot[1]**2), qdot[0]**2]).T)
    H1 = alpha3*np.sin(q2)*((-2*dq1*dq2)-(dq2**2))
    H2 = alpha3*np.sin(q2)*(dq1**2)
    C = alpha3*(np.array([-2*dq1*dq2 - dq2**2, dq1**2]).T)*np.sin(q2)
    # G = np.array([-beta1*np.sin(q[0]), -beta2*np.sin(q[1])]).T
    G = np.array([beta1*np.cos(q1) + beta2*np.cos(q1+q2), beta2*np.cos(q1+q2)]).T
    G1 = beta1*np.cos(q1) + beta2*np.cos(q1+q2)
    G2 = beta2*np.cos(q1+q2)
    qdot = np.array([dq1,dq2])
    E = 0.5*np.dot(np.dot(qdot.T, M),qdot) + beta1*np.sin(q1) + beta2*np.sin(q1+q2)
    delta = alpha1*alpha2 - (alpha3**2)*np.cos(q2)**2
    #E = 0.5*np.dot(qdot.T,np.dot(M,qdot)) + beta1*(np.cos(q[0]) - 1) + beta2*(np.cos(q[1]) - 1)
    # Er = -2*(beta1+beta2)
    Er = (beta1+beta2)
    N = (kv*dq2 + kp*q2)*delta + kd*(M21*(H1+G1) - M11*(H2+G2))
    D = kd*M11 + (E-Er)*delta
    #control_input = (np.dot(-kp,q[1]) - np.dot(kv,qdot[1]) + np.dot(np.dot(np.dot(kd,BA.T),np.linalg.inv(M)),(H+G)) ) / ((E-Er) + np.dot(np.dot(np.dot(kd,BA.T),np.linalg.inv(M)),(BA)))
    control_input = - ((N)/(D))
    global energy
    energy = E - Er
    return control_input

    

def simulate():
    simDT = 1/240 # simulation timestep   (was 1/240)
    simTime = 25 # total simulation time in seconds (was 25)

    # Initialize the current state
    current_state = np.array([0, 0, 0, 0])  # [q1, q2, dq1, dq2]
    #dot_current_state = np.array([0, 0])
    desired_state = np.array([np.pi/2, 0, 0, 0])  # [q1, q2, dq1, dq2]
    #dot_desired_state = np.array([-np.pi, 0])
    robotID, robotModel = sim_utils.simulationSetup(simDT)
    jointIndices = np.array([0,1])
    for i in jointIndices:
        pb.resetJointState(robotID, i, current_state[i])
    current_state[0] = current_state[0] % (2*np.pi)
    current_state[1] =  current_state[1] % (2*np.pi)
    dot_current_state = np.array([0,0])

    # Initialize the figure
    plt.ion()

    # Create the plots
    fig1, tauPlot = plt.subplots()
    fig2, energyPlot = plt.subplots()
    fig3, qPlot = plt.subplots()
    fig4, qdotPlot = plt.subplots()

    # Plot the initial empty data
    plot_utils.init_plot(tauPlot, 'Time [s]', '\u03C4' + '2 [Nm]', 'Time responses of ' + '\u03C4'+ '2 of the Acrobot in the swing-up phase')
    plot_utils.init_plot(energyPlot, 'Time [s]', 'E−Er [J]', 'Time responses of E of the Acrobot in the swing-up phase')
    plot_utils.init_plot(qPlot, 'Time [s]', '[rad/s]', 'Time responses of E of the Acrobot in the swing-up phase')
    plot_utils.init_plot(qdotPlot, 'Time [s]', '[rad]', 'Time responses of states of the Acrobot in the swing-up phase')

    # Create a directory for the files
    folder_path ='plots'
    if os.path.exists(folder_path):
        shutil.rmtree(folder_path)
        os.makedirs(folder_path, exist_ok=True)
    else:
        os.makedirs(folder_path, exist_ok=True)

    # Generate a unique filename for each plot
        filename1 = os.path.join(folder_path, f'torque')
        filename2 = os.path.join(folder_path, f'energy')
        filename3 = os.path.join(folder_path, f'q')
        filename4 = os.path.join(folder_path, f'qdot')

    # Initialize start time
    start_time = time.time()
    
    xValues = []
    tau_yValues = []
    energy_yValues = []
    
    # Control loop
    input("press ENTER to START the simulation:")
    for _ in range(int(simTime/simDT)):
        control_input =  calculate_control(current_state[0], current_state[1], dot_current_state[0], dot_current_state[1])# Apply control law, K is the controller gain matrix
        print("STO STAMPANDO IL TORQUE: \n")
        print(control_input)

        #Fill the x value with time and y value with torque
        current_time = time.time()
        time_diff = current_time - start_time
        #tauX = _ * 100  # Time in milliseconds
        x = time_diff
        tauY = control_input  # y-coordinate of the point
        energyY = energy

        xValues.append(x)
        tau_yValues.append(tauY)
        energy_yValues.append(energy)

        # Update the csv
        plot_utils.csv_write(x, tauY, filename1)
        plot_utils.csv_write(x, energyY, filename2)
        

        # Update the plot
        plot_utils.update_plot(tauPlot, xValues, tau_yValues, 'Time [s]', '\u03C4' + '2 [Nm]', 'Time responses of ' + '\u03C4'+ '2 of the Acrobot in the swing-up phase', filename1)
        plot_utils.update_plot(energyPlot, xValues, energy_yValues, 'Time [s]', 'E−Er [J]', 'Time responses of E of the Acrobot in the swing-up phase', filename2)
        plot_utils.update_2line_plot(qPlot, x, (current_state[0] - 1.57), current_state[1],  'Time [s]', '[rad]', 'q1-pi/2', 'q1', 'Time responses of states of the Acrobot in the swing-up phase',filename3)
        plot_utils.update_2line_plot(qdotPlot, x, dot_current_state[0], dot_current_state[1],  'Time [s]', '[rad/s]', 'q1dot', 'q1dot', 'Time responses of states of the Acrobot in the swing-up phase',filename4)
        

        #plt.pause(0.001)
        control_torque = np.array([0,control_input])
        pb.setJointMotorControlArray(robotID, jointIndices, controlMode = pb.TORQUE_CONTROL, forces=control_torque)
        if np.isnan(control_torque[1]):
            break
        pb.stepSimulation()
        time.sleep(simDT)
        # current_state, dot_current_state = sim_utils.getState(robotID, jointIndices)
        next_state = acrobot_dynamics(current_state[0], current_state[1], dot_current_state[0], dot_current_state[1], control_input, 1/240)  # Update the state based on dynamics
        current_state[0] = next_state[0]
        current_state[1] = next_state[1]
        dot_current_state[0] = next_state[2]
        dot_current_state[1] = next_state[3]
        
        current_state[0] = current_state[0] % (2*np.pi)
        current_state[1] =  current_state[1] % (2*np.pi)

    # Disable interactive mode after the loop
    plt.ioff()
    
    input("press ENTER to CLOSE the simulation:")

    pb.disconnect()

    