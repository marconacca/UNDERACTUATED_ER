import pybullet as pb
import time
import numpy as np
import pinocchio as pin
from controller_implementation import swing_up_control
from dynamic_scipy import integration
from plot_function import plotting_arrays, plotting_singleval
import matplotlib.pyplot as plt
from acro_dynamics import acrobot_dynamics

import sim_utils


def simulate():

    simDT = 1/240 # simulation timestep   (was 1/240)
    simTime = 20 # total simulation time in seconds (was 25)
    num_step = simTime / simDT

    robotID, robotModel = sim_utils.simulationSetup(simDT)
    nDof = 2

    # **********   SETTING INITIAL STATE   *********
    q0 = np.array([-1.4, 0]) # initial configuration
    qdot0 = np.array([0, 0]) # initial velocity
    state = np.array([q0[0], q0[1], qdot0[0], qdot0[1]])
    q = q0
    qdot = qdot0

    # set a DESIRED JOINT CONFIGURATION and VELOCITY
    qdes = np.array([np.pi/2,0])
    qdotdes = np.array([0,0])

    # Initialize arrays to stores important values and for plotting purposes
    q_history = np.array([q0])
    qdot_history = np.array([qdot0])
    torques_history = np.array([[0,0]])
    energy_error_history = np.array([0])
    sim_step = np.array([0])
    seconds = np.array([0])
    s = 0




    # @@@@@@@@@@   we are going to consider both revolute joints, so we fill the whole joint indices list @@@@@@@@@@
    jointIndices = np.array([0,1])
    #print('jointIndices :', jointIndices)

    for i in jointIndices:
        pb.resetJointState(robotID, i, q0[i])

    #q, qdot = sim_utils.getState(robotID, jointIndices)


    # # Get the transform of the World frame relative to itself
    # world_placement = robotModel.model.jointPlacements[0]
    # # Extract the position and orientation from the transform
    # world_position = world_placement.translation
    # world_orientation = world_placement.rotation
    # # Print the values of the World frame
    # print("World Frame:")
    # print("Position:", world_position)
    # print("Orientation:", world_orientation)


    # Compute the desired potential energy in vertical position

    # #Set joint positions and velocities
    # pin.forwardKinematics(robotModel.model, robotModel.data, qdes)
    # gravity = robotModel.model.gravity.linear[2]  # Assuming gravity is in the z-direction
    # des_potential_energy = 0.0
    # for i in range(robotModel.model.njoints):
    #     com_pos = robotModel.data.oMi[i].translation
    #     mass = robotModel.model.inertias[i].mass
    #     des_potential_energy += mass * gravity * com_pos[2]

    # energy_des = des_potential_energy



    # ********************   SIMULATION CYCLE   ********************

    input("press ENTER to START the simulation:")

    for i in range(int(num_step)):


        # read the current joint state from the simulator
        #q, qdot = sim_utils.getState(robotID, jointIndices)


        # # angle wrapping between 0 and 2pi
        #q = q % (2 * np.pi)
        #q = np.arctan2(np.sin(q), np.cos(q)) 
        #Define the maximum allowed velocity
        #max_velocity = 5.0
        #Clamp joint velocities to the range [-max_velocity, max_velocity]
        #joint_velocities_clamped = np.clip(qdot, -max_velocity, max_velocity)

        control_torques, energy_error = swing_up_control(robotModel, q, qdot, qdes, qdotdes)
        print('\n***** control_torques: ', control_torques)
        print('\n')


    
        # **********   compute Dynamics and our Euler Integration   **********
        qnext, qdotnext = integration(q, qdot, simDT ,control_torques)
        #qnext, qdotnext = acrobot_dynamics(q, qdot, control_torques, simDT )
        q = qnext
        qdot = qdotnext
        #q angle wrapping between 0 and 2pi  or  between -pi and pi
        #q = q % (2 * np.pi)
        #q = np.arctan2(np.sin(q), np.cos(q))
        
        # q = np.array([q[0]-np.pi/2, q[1] - q[0]])
        # qdot = np.array([qdot[0], qdot[1] - qdot[0]])
        
        
        print('***** states : ', q, qdot)
        print('\n')


        # STORE DATA for plotting
        if(i % 240 == 0):
            q_history = np.append(q_history, [q], axis=0)
            qdot_history = np.append(qdot_history, [qdot], axis=0)
            torques_history = np.append(torques_history, [control_torques], axis=0)
            energy_error_history = np.append(energy_error_history,energy_error)
            # i time step of simulation for plotting
            #sim_step = np.append(sim_step, i)
            seconds = np.append(seconds, s)
            s = s+1

        #if(abs(torques_history[i, 1]) > 3):
        #    input('Check values of control')


        pb.setJointMotorControlArray(robotID, jointIndices, controlMode = pb.TORQUE_CONTROL, forces=control_torques)
        # advance the simulation one step
        pb.stepSimulation()
        time.sleep(simDT)


    # @@@@@@@@@@@@@@@   PLOTS TESTING   @@@@@@@@@@@@@@@
    # plotting_arrays(sim_step, q_history, 'timestep', 'theta [rad]', ['q1', 'q2'], 'Configuration angle q wrt time')
    # plotting_arrays(sim_step, qdot_history, 'timestep', 'Vjoint [rad/s]', ['dq1', 'dq2'], 'Joint Velocity dq wrt time')
    # plotting_arrays(sim_step, torques_history, 'timestep', 'Torque [N*m]', ['Tau1', 'Tau2'], 'Torques Tau wrt time')
    # plotting_singleval(sim_step, energy_error_history, 'timestep', 'En.ERR [J]', ['En.ERR'], 'Energy error wrt time')
    plotting_arrays(seconds, q_history, 'timestep', 'theta [rad]', ['q1', 'q2'], 'Configuration angle q wrt time')
    plotting_arrays(seconds, qdot_history, 'timestep', 'Vjoint [rad/s]', ['dq1', 'dq2'], 'Joint Velocity dq wrt time')
    plotting_arrays(seconds, torques_history, 'timestep', 'Torque [N*m]', ['Tau1', 'Tau2'], 'Torques Tau wrt time')
    plotting_singleval(seconds, energy_error_history, 'timestep', 'En.ERR [J]', ['En.ERR'], 'Energy error wrt time')



    input("press ENTER to CLOSE the simulation:")

    pb.disconnect()

    return sim_step, q_history, qdot_history


