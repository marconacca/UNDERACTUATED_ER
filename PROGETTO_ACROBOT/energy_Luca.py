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
import math
from decimal import Decimal

energy = 0
def wrap_value(value, minimum, maximum):
    range_size = maximum - minimum + 1
    wrapped_value = (value - minimum) % range_size + minimum
    return wrapped_value
def calculate_control(q1, q2, dq1, dq2):
    # Define the control gains
    kp = 22    # Proportional gain
    kd = 15    # Dynamics gain
    kv = 45    # Derivative gain
    ke = 0.5

    l1 = 1
    l2 = 2
    lc1 = 0.5
    lc2 = 1
    m1 = 1
    m2 = 1
    I1 = 0.083
    I2 = 0.33
    g = 9.8

    q = np.array([q1-np.pi/2, q2-q1])
    qdot = np.array([dq1,dq2 - dq1])

    t1 = m1*(lc1*lc1) + m2*(l1*l1) + I1
    t2 = m2*(lc2*lc2) + I2
    t3 = m2*l1*lc2
    t4 = (m1*lc1 + m2*l1)
    t5 = m2*lc2
 

    M = np.array([[t1 + t2 + 2*t3*np.cos(q[1]), t2 + t3*np.cos(q[1])], [t2 + t3*np.cos(q[1]), t2]])
    print("STAMPO M: \n")
    print(M)
    M21 = t2 + t3*np.cos(q[1])
    M11 = t1 + t2 + 2*t3*np.cos(q[1])
    M22 = t2
    H1 = t3*np.sin(q[1])*((-2*qdot[0]*qdot[1])-(qdot[1]*qdot[1]))
    H2 = t3*np.sin(q[1])*(qdot[0]*qdot[0])
    # C = alpha3*(np.array([-2*qdot[0]*dq2 - dq2**2, dq1**2]).T)*np.sin(q2)
    G1 = t4*np.cos(q[0])*g + t5*np.cos(q[0]+q[1])*g
    G2 = t5*np.cos(q[0]+q[1])*g
    print("STAMPO M21: \n")
    print(M21)
    print("STAMPO M11: \n")
    print(M11)
    print("STAMPO M22: \n")
    print(M22)
    print("STAMPO H1: \n")
    print(H1)
    print("STAMPO H2: \n")
    print(H2)
    print("STAMPO G1: \n")
    print(G1)
    print("STAMPO G2: \n")
    print(G2)

    E = 0.5*np.dot(np.dot(qdot.T, M),qdot) + t4*np.sin(q[0])*g + t5*np.sin(q[0]+q[1])*g
    #delta = alpha1*alpha2 - (alpha3**2)*np.cos(q[1])**2
    delta = M11*M22 - (M21*M21)
    Er = (t4+t5)*g
    error = E - Er
    print("STAMPO L'ENERGIA DESIDERATA: \n")
    print(Er)
    print("STAMPO L'ERRORE: \n")
    print(error)
    N = -(kv*qdot[1] + kp*q[1])*delta - kd*(M21*(H1+G1) - M11*(H2+G2))
    D = kd*M11 + (error)*delta*ke
    #control_input = (np.dot(-kp,q[1]) - np.dot(kv,qdot[1]) + np.dot(np.dot(np.dot(kd,BA.T),np.linalg.inv(M)),(H+G)) ) / ((E-Er) + np.dot(np.dot(np.dot(kd,BA.T),np.linalg.inv(M)),(BA)))
    control_input = (N/D)
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
    plot_utils.init_plot(qPlot, 'Time [s]', '[rad]', 'Time responses of states of the Acrobot in the swing-up phase')
    plot_utils.init_plot(qdotPlot, 'Time [s]', '[rad/s]', 'Time responses of states of the Acrobot in the swing-up phase')

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
    q1_Values = []
    q2_Values = []
    q1dot_Values = []
    q2dot_Values = []
    
    # Control loop
    input("press ENTER to START the simulation:")
    for _ in range(int(simTime/simDT)):
        ##  ------- GET CURRENT STATE GIVEN BY FUNCTION "GETSTATE" -------
        # current_state, dot_current_state = sim_utils.getState(robotID, jointIndices)
        
        ##  ------- PRINT OF THE VALUES -------
        print("STAMPO Q1: \n")
        print(current_state[0])
        print("STAMPO Q2: \n")
        print(current_state[1])
        # dot_current_state[0] = wrap_value(dot_current_state[0], -5, 5)
        # dot_current_state[1] = wrap_value(dot_current_state[1], -1.5, 1.5)

        ##  ------- WRAPPING THE ANGLES -------
        # current_state[0] = current_state[0] % (2*np.pi)
        # current_state[1] = current_state[1] % (2*np.pi)

        print("STAMPO Q1 WRAPPED: \n")
        print(current_state[0])
        print("STAMPO Q2 WRAPPED: \n")
        print(current_state[1])


        control_input =  calculate_control(current_state[0], current_state[1], dot_current_state[0], dot_current_state[1])# Apply control law, K is the controller gain matrix
        print("STO STAMPANDO IL TORQUE: \n")
        print(control_input)

        ##  Fill the x value with time and y value with torque
        current_time = time.time()
        time_diff = current_time - start_time
        #tauX = _ * 100  # Time in milliseconds
        x = time_diff
        tauY = control_input  # y-coordinate of the point
        energyY = energy

        xValues.append(x)
        tau_yValues.append(tauY)
        energy_yValues.append(energy)
        q1_Values.append((current_state[0] - 1.57))
        q2_Values.append(current_state[1])
        q1dot_Values.append(dot_current_state[0])
        q2dot_Values.append(dot_current_state[1])


        #   Update the csv
        plot_utils.csv_write(x, tauY, filename1)
        plot_utils.csv_write(x, energyY, filename2)
        plot_utils.csv_write_multiple(x, (current_state[0]), current_state[1], filename3)
        plot_utils.csv_write_multiple(x, dot_current_state[0], dot_current_state[1], filename4)
        

        #   Update the plot
        plot_utils.update_plot(tauPlot, xValues, tau_yValues, 'Time [s]', '\u03C4' + '2 [Nm]', 'Time responses of ' + '\u03C4'+ '2 of the Acrobot in the swing-up phase', filename1)
        plot_utils.update_plot(energyPlot, xValues, energy_yValues, 'Time [s]', 'E−Er [J]', 'Time responses of E of the Acrobot in the swing-up phase', filename2)
        plot_utils.update_2line_plot(qPlot, xValues, q1_Values, q2_Values,  'Time [s]', '[rad]', 'q1', 'q2', 'Time responses of states of the Acrobot in the swing-up phase',filename3)
        plot_utils.update_2line_plot(qdotPlot, xValues, q1dot_Values, q2dot_Values,  'Time [s]', '[rad/s]', 'q1dot', 'q2dot', 'Time responses of states of the Acrobot in the swing-up phase',filename4)
        

        #   plt.pause(0.001)
        control_torque = np.array([0,control_input])
        pb.setJointMotorControlArray(robotID, jointIndices, controlMode = pb.TORQUE_CONTROL, forces=control_torque)
        if np.isnan(control_torque[1]):
            break
        pb.stepSimulation()
        time.sleep(simDT)
        
        ##  ------- CALCULATE THE NEXT STATE WITH THE INTEGRATION -------
        next_state = acrobot_dynamics(current_state[0], current_state[1], dot_current_state[0], dot_current_state[1], control_input, int(simTime/simDT), robotModel)  # Update the state based on dynamics
        current_state[0] =  np.int64(next_state[0])
        current_state[1] =  np.int64(next_state[1])
        dot_current_state[0] =  np.int64(next_state[2])
        dot_current_state[1] =  np.int64(next_state[3])
        if _ == 4:
            break

    # Disable interactive mode after the loop
    plt.ioff()
    
    input("press ENTER to CLOSE the simulation:")

    pb.disconnect()

    