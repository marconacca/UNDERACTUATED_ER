import pybullet as pb
import time
import numpy as np
import pinocchio as pin
import plot_utils
import matplotlib.pyplot as plt
import shutil
import os
from controller_implementation import *
from dynamic_scipy import integration
from acro_dynamics import *

import sim_utils



def simulate():

    # /////////////////////   SIMULATION SETTINGS   /////////////////////
    simDT = 1/240 # simulation timestep 
    simTime = 25 # total simulation time in seconds

    q0 = np.array([-1.4, 0]) # initial configuration
    qdot0 = np.array([0, 0]) # initial velocity
    initial_state = np.array([q0[0], q0[1], qdot0[0], qdot0[1]])  # Initial state [q1, q2, dq1, dq2]

    qdes = np.array([np.pi/2,0]) # desired configuration
    qdotdes = np.array([0,0]) # desired velocity
    desired_state = np.array([np.pi/2, 0, 0, 0])  # Desired state for stabilization [q1, q2, dq1, dq2]

    robotID, robotModel = sim_utils.simulationSetup(simDT)

    nDof = 2

    # we are going to consider both revolute joints, so we fill the whole
    # joint indices list
    jointIndices = range(nDof)

    for i in jointIndices:
        pb.resetJointState(robotID, i, q0[i])

    q, qdot = sim_utils.getState(robotID, jointIndices) 

    q1_collection = []
    q2_collection = []
    q1dot_collection = []
    q2dot_collection = []
    taus_collection = []
    energy_collection = []


    # /////////////////////   PLOTS SETTINGS  /////////////////////
    
    #plt.ion() # Initialize the figure

    # Create the plots
    #fig1, tauPlot = plt.subplots()
    #fig2, energyPlot = plt.subplots()
    #fig3, qPlot = plt.subplots()
    #fig4, qdotPlot = plt.subplots()

    # Plot the initial empty data
    #plot_utils.init_plot(tauPlot, 'Time [s]', '\u03C4' + '2 [Nm]', 'Time responses of ' + '\u03C4'+ '2 of the Acrobot in the swing-up phase')
    #plot_utils.init_plot(energyPlot, 'Time [s]', 'E−Er [J]', 'Time responses of E of the Acrobot in the swing-up phase')
    #plot_utils.init_plot(qPlot, 'Time [s]', '[rad]', 'Time responses of states of the Acrobot in the swing-up phase')
    #plot_utils.init_plot(qdotPlot, 'Time [s]', '[rad/s]', 'Time responses of states of the Acrobot in the swing-up phase')

    # /////////////////////   FILE.csv SETTINGS  /////////////////////
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
    time_x_collection = []

    input("press ENTER to START the simulation:")

    for i in range(int(simTime/simDT)):

        current_time = time.time()
        time_diff = current_time - start_time
        x = time_diff

        # read the current joint state from the simulator
        #q, qdot = sim_utils.getState(robotID, jointIndices)    

        # compute the feedback torque command
        torques, energy_error = swing_up_control(robotModel, q, qdot)
        #if (switch(q, qdot) and i != 0):
            #torques,state_error = stabilization_control(robotModel, q, qdot, qdes, qdotdes)
            


        

        time_x_collection.append(x)
        taus_collection.append(torques[1])
        energy_collection.append(energy_error)
        q1_collection.append(q[0] - ((np.pi / 2) % np.pi))
        q2_collection.append(q[1])
        q1dot_collection.append(qdot[0])
        q2dot_collection.append(qdot[1])

        # send the torque command to the simulator
        pb.setJointMotorControlArray(robotID, jointIndices, controlMode = pb.TORQUE_CONTROL, forces = torques)

        # Dynamics and our Euler Integration  
        q_next,qdot_next = acrobot_dynamics(q, qdot, torques, simDT)
        #store the new state
        q = q_next
        qdot = qdot_next

        


        # Update the csv
        plot_utils.csv_write(x, torques[1], filename1)
        plot_utils.csv_write(x, energy_error, filename2)
        plot_utils.csv_write_multiple(x, (q[0] - ((np.pi / 2) % np.pi)), q[1], filename3)
        plot_utils.csv_write_multiple(x, qdot[0], qdot[1], filename4)
        

        # Update the plot
        #plot_utils.update_plot(tauPlot, time_x_collection, taus_collection, 'Time [s]', '\u03C4' + '2 [Nm]', 'Time responses of ' + '\u03C4'+ '2 of the Acrobot in the swing-up phase', filename1)
        #plot_utils.update_plot(energyPlot, time_x_collection, energy_collection, 'Time [s]', 'E−Er [J]', 'Time responses of E of the Acrobot in the swing-up phase', filename2)
        #plot_utils.update_2line_plot(qPlot, time_x_collection, q1_collection, q2_collection,  'Time [s]', '[rad]', 'q1-pi/2', 'q2', 'Time responses of states of the Acrobot in the swing-up phase',filename3)
        #plot_utils.update_2line_plot(qdotPlot, time_x_collection, q1dot_collection, q2dot_collection,  'Time [s]', '[rad/s]', 'q1dot', 'q1dot', 'Time responses of states of the Acrobot in the swing-up phase',filename4)

        # advance the simulation one step
        pb.stepSimulation()
        time.sleep(simDT)

    # Disable interactive mode after the loop
    #plt.ioff()

    input("press ENTER to CLOSE the simulation:")

    pb.disconnect()





