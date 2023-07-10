import pybullet as pb
import time
import numpy as np
import pinocchio as pin
from controller_implementation import swing_up_control
from dynamic_scipy import integration

import sim_utils


def simulate():

    simDT = 1/240 # simulation timestep   (was 1/240)
    simTime = 25 # total simulation time in seconds (was 25)
    q0 = np.array([0, 0]) # initial configuration

    robotID, robotModel = sim_utils.simulationSetup(simDT)

    nDof = 2

    # we are going to consider both revolute joints, so we fill the whole
    # joint indices list
    jointIndices = np.array([0,1])
    print('jointIndices :', jointIndices)

    for i in jointIndices:
        pb.resetJointState(robotID, i, q0[i])

    q, qdot = sim_utils.getState(robotID, jointIndices)
    # print(controller.computeEEpose(robotModel, q))



    # # Get the transform of the World frame relative to itself
    # world_placement = robotModel.model.jointPlacements[0]

    # # Extract the position and orientation from the transform
    # world_position = world_placement.translation
    # world_orientation = world_placement.rotation

    # # Print the values of the World frame
    # print("World Frame:")
    # print("Position:", world_position)
    # print("Orientation:", world_orientation)


    # set a desired joint configuration
    qdes = np.array([np.pi/2,0])
    qdotdes = np.array([0,0])
    #edes = 3.0

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
    # set a desired ee configuration
    # ee_des = np.array([0, 0.3])



    # ********************   SIMULATION CYCLE   ********************

    input("press ENTER to START the simulation:")

    for i in range(int(simTime/simDT)):


        # read the current joint state from the simulator
        #q, qdot = sim_utils.getState(robotID, jointIndices)

        # # angle wrapping between 0 and 2pi
        # joint_angles_wrapped = q % (2 * np.pi) 
        # #Define the maximum allowed velocity
        # max_velocity = 5.0
        # #Clamp joint velocities to the range [-max_velocity, max_velocity]
        # joint_velocities_clamped = np.clip(qdot, -max_velocity, max_velocity)

        control_torques = swing_up_control(robotModel, q, qdot, qdes, qdotdes)
        print('\n***** control_torques: ', control_torques)
        print('\n')

        # compute dynamics
        q, qdot = integration(q, qdot, simDT ,control_torques)
        # q angle wrapping between 0 and 2pi
        #q = q % (2 * np.pi) 
        
        print('***** states : ', q, qdot)
        print('\n')

        pb.setJointMotorControlArray(robotID, jointIndices, controlMode = pb.TORQUE_CONTROL, forces=control_torques)
        # advance the simulation one step
        pb.stepSimulation()
        time.sleep(simDT)
        #input('STO A FA LA COLLA!!\n')


    input("press ENTER to CLOSE the simulation:")

    pb.disconnect()





