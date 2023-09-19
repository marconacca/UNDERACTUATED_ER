import os
import shutil
import pybullet as pb
import time
import numpy as np
import pinocchio as pin
from controller_implementation import swing_up_control
from dynamic_scipy import integration
from plot_function import plotting_conf, plotting_vel, plotting_torque, plotting_singleval
import matplotlib.pyplot as plt
from acro_dynamics import acrobot_dynamics

import sim_utils

#for the graphical part
import sys
import matplotlib.pyplot as plt
from matplotlib.patches import Circle



def simulate():

    simDT = 1/240 # simulation timestep   (was 1/240)
    simTime = 25 # total simulation time in seconds (was 25)
    num_step = int(simTime / simDT)

    #robotID, robotModel = sim_utils.simulationSetup(simDT)
    #nDof = 2

    # **********   SETTING INITIAL STATE   *********
    q0 = np.array([-1.4, 0.0]) # initial configuration paper 2007
    #q0 = np.array([np.pi/2.-1.4, 0.0]) # initial configuration another paper
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
    q_history = np.array([q0])
    qdot_history = np.array([qdot0])
    torques_history = np.array([[0,0]])
    energy_error_history = np.array([0])
    sim_step = np.array([0])
    seconds = np.array([0])
    s = 0




    # we are going to consider both revolute joints, so we fill the whole joint indices list
    #jointIndices = np.array([0,1])
    #print('jointIndices :', jointIndices)

    #for i in jointIndices:
        #pb.resetJointState(robotID, i, q0[i])


    # found in code of our paper but it's equal to modify the initial conition
    # q[1] = q[1] - q[0]
    # q[0] = q[0] - np.pi/2
    # qdot[1] = qdot[1] - qdot[0]

    # ********************   SIMULATION CYCLE   ********************
    folder_path = 'plots'
    if os.path.exists(folder_path):
        shutil.rmtree(folder_path)
        os.makedirs(folder_path, exist_ok=True)
    else:
        os.makedirs(folder_path, exist_ok=True)

    input("press ENTER to START the simulation:")

    for i in range(int(num_step)):


        # read the current joint state from the simulator
        #q, qdot = sim_utils.getState(robotID, jointIndices)


        control_torques, energy_error = swing_up_control(q, qdot, initial_state, desired_state)
        print('\n***** control_torques: ', control_torques)
        print('\n')


    
        # **********   compute Dynamics and our Euler Integration   **********
        #qnext, qdotnext = integration(q, qdot, simDT ,control_torques)
        qnext, qdotnext = acrobot_dynamics(q, qdot, control_torques, simDT )
        q = qnext
        qdot = qdotnext
        
        
        # **********   Angle wrapping   **********
        #  q angle wrapping between 0 and 2pi
        #q = q % (2 * np.pi)
        
        #  angle wrapping between -pi and pi
        #q = np.arctan2(np.sin(q), np.cos(q))
        #q[0] = np.arctan2(np.sin(q[0]), np.cos(q[0]))
        
        #  another angle wrapping of similar paper
        #q[0] = q[0] % (2*np.pi)
        #q[1] = (q[1] + np.pi) % (2*np.pi) - np.pi
        
        
        # ----- Define the maximum allowed velocity
        #max_velocity = 5.0
        #Clamp joint velocities to the range [-max_velocity, max_velocity]
        #joint_velocities_clamped = np.clip(qdot, -max_velocity, max_velocity)
        
        
        print('***** states : ', q, qdot)
        print('\n')


        # STORE DATA for plotting
        if(i % 1 == 0):
            q_history = np.append(q_history, [q], axis=0)
            qdot_history = np.append(qdot_history, [qdot], axis=0)
            torques_history = np.append(torques_history, [control_torques], axis=0)
            energy_error_history = np.append(energy_error_history,energy_error)
            # i time step of simulation for plotting
            #sim_step = np.append(sim_step, i)
            seconds = np.append(seconds, s)
            s = s+1

        #if(abs(torques_history[i, 1]) > 25):
        #    input('Check values of control')


        #pb.setJointMotorControlArray(robotID, jointIndices, controlMode = pb.TORQUE_CONTROL, forces=control_torques)
        # advance the simulation one step
        #pb.stepSimulation()
        #time.sleep(simDT)



    # @@@@@@@@@@@@@@@@@@@@@@@@@    PLOTS TESTING    @@@@@@@@@@@@@@@@@@@@@@@@@
    # plotting_arrays(sim_step, q_history, 'timestep', 'theta [rad]', ['q1', 'q2'], 'Configuration angle q wrt time')
    # plotting_arrays(sim_step, qdot_history, 'timestep', 'Vjoint [rad/s]', ['dq1', 'dq2'], 'Joint Velocity dq wrt time')
    # plotting_arrays(sim_step, torques_history, 'timestep', 'Torque [N*m]', ['Tau1', 'Tau2'], 'Torques Tau wrt time')
    # plotting_singleval(sim_step, energy_error_history, 'timestep', 'En.ERR [J]', ['En.ERR'], 'Energy error wrt time')
    plotting_conf(seconds, q_history, 'timestep', 'theta [rad]', ['q1', 'q2'], 'Configuration angle q wrt time')
    plotting_vel(seconds, qdot_history, 'timestep', 'Vjoint [rad/s]', ['dq1', 'dq2'], 'Joint Velocity dq wrt time')
    plotting_torque(seconds, torques_history, 'timestep', 'Torque [N*m]', ['Tau1', 'Tau2'], 'Torques Tau wrt time')
    plotting_singleval(seconds, energy_error_history, 'timestep', 'En.ERR [J]', ['En.ERR'], 'Energy error wrt time')



    #input("press ENTER to CLOSE the simulation:")

    #pb.disconnect()

    return num_step, q_history, qdot_history




def make_plot(i, ax, x1, x2, y1, y2, L1, L2, simDT, di):
    # Plot and save an image of the double pendulum configuration for time
    # point i.
    # The pendulum rods.
    

    # Plotted bob circle radius
    r = 0.05
    # Plot a trail of the m2 bob's position for the last trail_secs seconds.
    trail_secs = 1
    # This corresponds to max_trail time points.
    max_trail = int(trail_secs / simDT)
    

    ax.plot([0, x1[i], x2[i]], [0, y1[i], y2[i]], lw=2, c='k')
    # Circles representing the anchor point of rod 1, and bobs 1 and 2.
    c0 = Circle((0, 0), r/2, fc='k', zorder=10)
    c1 = Circle((x1[i], y1[i]), r, fc='b', ec='b', zorder=10)
    c2 = Circle((x2[i], y2[i]), r, fc='r', ec='r', zorder=10)

    ax.add_patch(c0)
    ax.add_patch(c1)
    ax.add_patch(c2)

    # The trail will be divided into ns segments and plotted as a fading line.
    ns = 20
    s = max_trail // ns

    for j in range(ns):
        imin = i - (ns-j)*s
        if imin < 0:
            continue
        imax = imin + s + 1
        # The fading looks better if we square the fractional length along the
        # trail.
        alpha = (j/ns)**2
        ax.plot(x2[imin:imax], y2[imin:imax], c='r', solid_capstyle='butt',
                lw=2, alpha=alpha)

    # Centre the image on the fixed anchor point, and ensure the axes are equal
    ax.set_xlim(-L1-L2-r, L1+L2+r)
    ax.set_ylim(-L1-L2-r, L1+L2+r)
    ax.set_aspect('equal', adjustable='box')
    plt.axis('off')
    plt.savefig('frames/_img{:04d}.png'.format(i//di), dpi=72)
    plt.cla()
