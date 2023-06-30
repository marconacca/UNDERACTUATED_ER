import pybullet as pb
import time
import numpy as np
import pinocchio as pin
from energy_analysis import compute_energy
from controller_implementation import swing_up_control

import sim_utils
# import controller
import controller_implementation

def simulate():

    simDT = 1/240 # simulation timestep
    simTime = 25 # total simulation time in seconds
    q0 = np.array([np.pi/2, 0]) # initial configuration

    robotID, robotModel = sim_utils.simulationSetup(simDT)

    nDof = 2

    # we are going to consider both revolute joints, so we fill the whole
    # joint indices list
    jointIndices = range(nDof)

    for i in jointIndices:
        pb.resetJointState(robotID, i, q0[i])

    q, qdot = sim_utils.getState(robotID, jointIndices) 

    # print(controller.computeEEpose(robotModel, q))

    # in general we need to call this to compute all the kinematic and 
    # dynamic quantities (see pinocchio docs) which can be retrieved 
    # either as members of the robotModel.data object, or via specific
    # functions
    # pin.computeAllTerms(robotModel.model, robotModel.data, q, qdot)

    # set a desired joint configuration
    qdes = np.array([0,np.pi/2])
    edes = 3
    # set a desired ee configuration
    # ee_des = np.array([0, 0.3])

    input("press ENTER to START the simulation:")

    for i in range(int(simTime/simDT)):

        """ # read the current joint state from the simulator
        q, qdot = sim_utils.getState(robotID, jointIndices)    

        # compute the feedback torque command
        tau = controller.JointPositionControl(robotModel, q, qdot, qdes)
        #tau = controller.EePositionController(robotModel, q, qdot, ee_des)

        # send the torque command to the simulator
        pb.setJointMotorControlArray(
            robotID, jointIndices, controlMode = pb.TORQUE_CONTROL, forces = tau)

        # advance the simulation one step
        pb.stepSimulation()
        time.sleep(simDT) """
        # read the current joint state from the simulator
        q, qdot = sim_utils.getState(robotID, jointIndices)    
        control_torques = swing_up_control(robotModel, q, qdot, edes, qdes)
        pb.setJointMotorControlArray(robotID, jointIndices, controlMode = pb.TORQUE_CONTROL, forces=control_torques)
        # advance the simulation one step
        pb.stepSimulation()
        time.sleep(simDT)


    input("press ENTER to CLOSE the simulation:")

    pb.disconnect()





