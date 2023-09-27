import os
import shutil
import pybullet as pb
import time
import numpy as np
import pinocchio as pin
from controller_implementation import swing_up_control
from dynamic_scipy import integration
from plot_utils import *

#for the graphical part
import sys
import matplotlib.pyplot as plt
from matplotlib.patches import Circle



def simulate():

    simDT = 1/240 # simulation timestep
    simTime = 10 # total simulation time in seconds
    num_step = int(simTime / simDT)

    #robotID, robotModel = sim_utils.simulationSetup(simDT)
    #nDof = 2

    # **********   SETTING INITIAL STATE   *********
    q0 = np.array([-1.4046, 0.0]) # initial configuration paper 2007
    #q0 = np.array([np.pi/2.-1.4, 0.0]) # initial configuration Working code
    qdot0 = np.array([0.0, 0.0]) # initial velocity
    
    initial_state = np.concatenate((q0, qdot0), axis=None)
    
    q = q0
    qdot = qdot0
    

    # set a DESIRED JOINT CONFIGURATION and VELOCITY
    qdes = np.array([np.pi/2,0.])
    #qdes = np.array([np.pi,0.])
    qdotdes = np.array([0.,0.])
    
    desired_state = np.concatenate((qdes, qdotdes), axis=None)
    

    # Initialize arrays to stores important values and for plotting purposes
    q_history = []
    q_pi2_history = []
    q1_pi2_history = []
    qdot_history = []
    q1dot_history = []
    torques_history = []
    torque_history = []
    energy_error_history = []
    state_history = []
    seconds = []
    s = 0




    # we are going to consider both revolute joints, so we fill the whole joint indices list
    #jointIndices = np.array([0,1])
    #print('jointIndices :', jointIndices)

    #for i in jointIndices:
        #pb.resetJointState(robotID, i, q0[i])





#_____________________________________                    ____________________________________
#____________________________________   SIMULATION CYCLE   __________________________________
#_____________________________________                    ____________________________________
    plots_path = 'plots'
    csv_folder = 'csv'
    frames_path = 'frames'

    clear_folder([plots_path, csv_folder, frames_path])

    input("press ENTER to START the simulation:")

    for i in range(int(num_step)):


        # read the current joint state from the simulator
        #q, qdot = sim_utils.getState(robotID, jointIndices)


        control_torques, energy_error = swing_up_control(q, qdot, initial_state, desired_state)
        print('\n***** control_torques: ', control_torques)
        print('\n')

        # STORE DATA for plotting
        
        q_history.append(q)
        q_pi2_history.append([q[0] - np.pi/2, q[1]])
        q1_pi2_history.append(q[0] - np.pi/2)
        qdot_history.append(qdot)
        q1dot_history.append(qdot[0])
        torques_history.append(control_torques)
        torque_history.append(control_torques[1])
        energy_error_history.append(energy_error)
        tmp_state = np.concatenate((q, qdot), axis = 0)
        state_history.append(tmp_state)
        seconds.append(s)
        s = s+1


    
        # **********   compute Dynamics and our Euler Integration   **********
        qnext, qdotnext = integration(q, qdot, simDT ,control_torques)
        q = qnext
        qdot = qdotnext
        
        
        # **********   Angle wrapping   **********
        #  q angle wrapping between 0 and 2pi
        #q = q % (2 * np.pi)
        
        #  angle wrapping between -pi and pi
        #q = np.arctan2(np.sin(q), np.cos(q))
        #q[0] = np.arctan2(np.sin(q[0]), np.cos(q[0]))
        
        #  another angle wrapping of Working code
        #q[0] = q[0] % (2*np.pi)
        #q[1] = (q[1] + np.pi) % (2*np.pi) - np.pi
        
        
        # ----- Define the maximum allowed velocity
        #max_velocity = 5.0
        #Clamp joint velocities to the range [-max_velocity, max_velocity]
        #joint_velocities_clamped = np.clip(qdot, -max_velocity, max_velocity)
        
        
        print('***** states : ', q, qdot)
        print('\n')


        


        # PYBULLET SIMULATION (Not Used)
        #pb.setJointMotorControlArray(robotID, jointIndices, controlMode = pb.TORQUE_CONTROL, forces=control_torques)
        # advance the simulation one step
        #pb.stepSimulation()
        #time.sleep(simDT)



    # @@@@@@@@@@@@@@@@@@@@@@@@@    PLOTS TESTING    @@@@@@@@@@@@@@@@@@@@@@@@@
    
    multi_line_plot(seconds, q_pi2_history, 'timestep', '[rad]', ['q\u2081-\u03C0/2', 'q\u2082'], 'Configuration angle q wrt time', 'q', plots_path)
    multi_line_plot(seconds, qdot_history, 'timestep', '[rad/s]', ['q\u2081dot', 'q\u2082dot'], 'Joint Velocity dq wrt time', 'qdot', plots_path)
    #multi_line_plot(seconds, torques_history, 'timestep', 'Torque [N*m]', ['\u03C4\u2081', '\u03C4\\u2082'], 'Torques Tau wrt time', 'torque', plots_path)
    single_line_plot(seconds, torque_history, 'timestep', '\u03C4\u2082[Nm]', '\u03C4\u2082', 'Torques Tau wrt time', 'torque', plots_path)
    single_line_plot(seconds, energy_error_history, 'timestep', 'E-Er[J]', 'E-Er', 'Energy error wrt time', 'er', plots_path)
    single_line_plot(q1_pi2_history, q1dot_history, 'q\u2081-\u03C0/2 [rad]', 'q\u2081dot [rad/s]', 'q\u2081-\u03C0/2', 'Phase portrait of q\u2081-\u03C0/2, q\u2081dot in the swing-up phase', 'phase', plots_path)



    #input("press ENTER to CLOSE the simulation:")

    #pb.disconnect()


    return num_step, q_history, qdot_history, state_history


def clear_folder(folders):
    for i in folders:
        if os.path.exists(i):
            shutil.rmtree(i)
            os.makedirs(i, exist_ok=True)
        else:
            os.makedirs(i, exist_ok=True)
