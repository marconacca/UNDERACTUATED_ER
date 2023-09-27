import pybullet as pb
import time
import numpy as np
import pinocchio as pin

import sim_utils

def simulate(state_collection, simDT, simTime):

    state_collection = np.array(state_collection)
    # simDT simulation timestep
    # simTime total simulation time in seconds

    q0 = np.array([-1.4046, 0.0]) # initial configuration

    robotID, robotModel = sim_utils.simulationSetup(simDT)

    nDof = 2

    # we are going to consider both revolute joints, so we fill the whole
    # joint indices list
    jointIndices = range(nDof)

    for i in jointIndices:
        pb.resetJointState(robotID, i, q0[i])

    #q, qdot = sim_utils.getState(robotID, jointIndices) 

    input("press ENTER to START the simulation:")

    for i in range(int(simTime/simDT)):

        for j in jointIndices:
            pb.resetJointState(robotID, j, state_collection[i][j], state_collection[i][j+2])

        # read the current joint state from the simulator
        #q, qdot = sim_utils.getState(robotID, jointIndices)    

        # send the torque command to the simulator
        #pb.setJointMotorControlArray(
            #robotID, jointIndices, controlMode = pb.TORQUE_CONTROL, forces = tau)

        # advance the simulation one step
        pb.stepSimulation()
        time.sleep(simDT)

    pb.disconnect()