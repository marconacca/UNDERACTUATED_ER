import pybullet as pb
import time
import numpy as np
import pinocchio as pin
import plot_utils
import matplotlib.pyplot as plt
import shutil
import os
from controller_implementation import *
from acro_dynamics2001 import *
from wrap_utils import *
import sim_utils



def simulate():

    # /////////////////////   SIMULATION SETTINGS   /////////////////////
    simDT = 1/240 # simulation timestep 
    simTime = 25 # total simulation time in seconds

    q0 = np.array([-np.pi, 0]) # initial configuration
    qdot0 = np.array([0, 0]) # initial velocity
    initial_state = np.array([q0[0], q0[1], qdot0[0], qdot0[1]])  # Initial state [q1, q2, dq1, dq2]

    qdes = np.array([0,0]) # desired configuration
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

    energy_error = 0


    # /////////////////////   PLOTS SETTINGS  /////////////////////
    
    #plt.ion() # Initialize the figure

    # Create the plots
    fig1, tauPlot = plt.subplots()
    fig2, energyPlot = plt.subplots()
    fig3, qPlot = plt.subplots()
    fig4, qdotPlot = plt.subplots()

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
    s = 0

    #q[1] = q[1] - q[0]
    #q[0] = q[0] - np.pi/2
    #qdot[1] = qdot[1] - qdot[0]

    g = 9.81

    ls = np.array([1.0,2.0])
    ms = np.array([1.0,1.0])
    lc = np.array([0.5,1.0])
    Is = np.array([0.083,0.33])

    t1 = ms[0]*lc[0]*lc[0] + ms[1]*ls[0]*ls[0] + Is[0]
    t2 = ms[1]*lc[1]*lc[1] + Is[1]
    t3 = ms[1]*ls[0]*lc[1]
    t4 = ms[0]*lc[0] + ms[1]*ls[0]
    t5 = ms[1]*lc[1]

    ts = np.array([t1, t2, t3, t4,t5]) #note in this case beta are without g

    kv = 45.0
    ke = 0.5
    kd = 15.0
    kp = 22.0

    ks = np.array([kv, ke, kd, kp])
    

    input("press ENTER to START the simulation:")

    for i in range(int(simTime/simDT)):
        x = s 
        state = np.concatenate((q, qdot))
        q_next, qdot_next, torques, energy_error = advance(state, ts, ks, simDT, ls, ms, lc, Is, g)

        # send the torque command to the simulator
        pb.setJointMotorControlArray(robotID, jointIndices, controlMode = pb.TORQUE_CONTROL, forces = torques)
            


        

        time_x_collection.append(x)
        taus_collection.append(torques[1])
        energy_collection.append(energy_error)
        #q1_collection.append(q[0] - ((np.pi / 2) % np.pi))
        q1_collection.append(q[0])
        q2_collection.append(q[1])
        q1dot_collection.append(qdot[0])
        q1dot_collection.append(qdot[1])
        
        
        
        updated_state = np.concatenate((q_next, qdot_next))
        
        #store the new state
        q = np.array(q_next)
        #q = wrap_angles_top(q)
        qdot = np.array(qdot_next)

        s = s+1

        


        # Update the csv
        plot_utils.csv_write(x, torques[1], filename1)
        plot_utils.csv_write(x, energy_error, filename2)
        plot_utils.csv_write_multiple(x, q[0], q[1], filename3)
        plot_utils.csv_write_multiple(x, qdot[0], qdot[1], filename4)
        

        # advance the simulation one step
        pb.stepSimulation()
        time.sleep(simDT)

    # Update the plot
    plot_utils.update_plot(tauPlot, time_x_collection, taus_collection, 'Time [s]', '\u03C4' + '2 [Nm]', 'Time responses of ' + '\u03C4'+ '2 of the Acrobot in the swing-up phase', filename1)
    plot_utils.update_plot(energyPlot, time_x_collection, energy_collection, 'Time [s]', 'E−Er [J]', 'Time responses of E of the Acrobot in the swing-up phase', filename2)
    #plot_utils.update_2line_plot(qPlot, time_x_collection, q1_collection, q2_collection,  'Time [s]', '[rad]', 'q1-pi/2', 'q2', 'Time responses of states of the Acrobot in the swing-up phase',filename3)
    #plot_utils.update_2line_plot(qdotPlot, time_x_collection, q1dot_collection, q2dot_collection,  'Time [s]', '[rad/s]', 'q1dot', 'q2dot', 'Time responses of states of the Acrobot in the swing-up phase',filename4)

    fig1.show()
    fig2.show()
    fig3.show()
    fig4.show()

    # Disable interactive mode after the loop
    #plt.ioff()

    input("press ENTER to CLOSE the simulation:")

    pb.disconnect()