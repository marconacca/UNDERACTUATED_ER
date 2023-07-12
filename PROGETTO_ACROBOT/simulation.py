import pybullet as pb
import time
import numpy as np
import pinocchio as pin
from controller_implementation import swing_up_control
from dynamic_scipy import integration
#from acro_dynamics import acrobot_dynamics

import sim_utils


def simulate():

    simDT = 1/240 # simulation timestep   (was 1/240)
    simTime = 25 # total simulation time in seconds (was 25)
    num_step = simTime / simDT

    robotID, robotModel = sim_utils.simulationSetup(simDT)
    nDof = 2

    # **********   Setting Initial State   *********
    q0 = np.array([1.5, 0]) # initial configuration
    qdot0 = np.array([0, 0]) # initial velocity
    state = np.array([q0[0], q0[1], qdot0[0], qdot0[1]])

    # we are going to consider both revolute joints, so we fill the whole
    # joint indices list
    jointIndices = np.array([0,1])
    print('jointIndices :', jointIndices)

    for i in jointIndices:
        pb.resetJointState(robotID, i, q0[i])

    q, qdot = sim_utils.getState(robotID, jointIndices)

    # set a desired joint configuration
    qdes = np.array([1.57,0])
    qdotdes = np.array([0,0])


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

    # Set joint positions and velocities
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
        q, qdot = sim_utils.getState(robotID, jointIndices)

        # # angle wrapping between 0 and 2pi
        #joint_angles_wrapped = q % (2 * np.pi)
        joint_angles_wrapped = np.arctan2(np.sin(q), np.cos(q)) 
        #Define the maximum allowed velocity
        max_velocity = 50.0
        #Clamp joint velocities to the range [-max_velocity, max_velocity]
        joint_velocities_clamped = np.clip(qdot, -max_velocity, max_velocity)

        control_torques = swing_up_control(robotModel, joint_angles_wrapped, joint_velocities_clamped, qdes, qdotdes)
        print('\n***** control_torques: ', control_torques)
        print('\n')

        # compute dynamics
        # qnext, qdotnext = integration(q, qdot, simDT ,control_torques)
        # #next_state = acrobot_dynamics(state, control_torques, simDT )
        #q, qdot = next_state
        # q angle wrapping between 0 and 2pi
        #q = q % (2 * np.pi) 
        # q = qnext
        # qdot = qdotnext
        
        # print('***** states : ', q, qdot)
        # print('\n')

        pb.setJointMotorControlArray(robotID, jointIndices, controlMode = pb.TORQUE_CONTROL, forces=control_torques)
        # advance the simulation one step
        pb.stepSimulation()
        time.sleep(simDT)
        #input('STO A FA LA COLLA!!\n')


    input("press ENTER to CLOSE the simulation:")

    pb.disconnect()


