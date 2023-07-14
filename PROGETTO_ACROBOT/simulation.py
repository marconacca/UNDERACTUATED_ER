import pybullet as pb
import time
import numpy as np
import pinocchio as pin
from controller_implementation import swing_up_control
from dynamic_scipy import integration
from acro_dynamics import *

import sim_utils


def simulate():

    simDT = 1/240 # simulation timestep   (was 1/240)
    simTime = 25 # total simulation time in seconds (was 25)

    q_curr = np.array([-np.pi,0])
    qdot_curr = np.array([0,0])
    qdes_curr = np.array([0,0])
    qdotdes_curr = np.array([0,0])
        # we are going to consider both revolute joints, so we fill the whole
    # joint indices list
    jointIndices = np.array([0,1])
    robotID, robotModel = sim_utils.simulationSetup(simDT)

    print('jointIndices :', jointIndices)
    for i in jointIndices:
        pb.resetJointState(robotID, i, q_curr[i])

    q_curr = q_curr % (2 * np.pi)
    qdot_curr = qdot_curr % (2 * np.pi)
    qdes_curr = qdes_curr % (2 * np.pi)
    qdotdes_curr = qdotdes_curr % (2 * np.pi)


    # ********************   SIMULATION CYCLE   ********************

    input("press ENTER to START the simulation:")

    for i in range(int(simTime/simDT)):
        q = np.concatenate([q_curr, qdot_curr])
        q_des = np.concatenate([qdes_curr, qdotdes_curr])
        errorq1 = q_des[0] - q[0]
        errorq2 = q_des[1] - q[1]
        if (abs(errorq1) < 0.2) and (abs(errorq2) < 0.2):
            input("LQR-CONTROL:    press ENTER to continue:")
            for _ in range(int(simTime/simDT)):
                error = q_des - q
                print("STO STAMPANDO L'ERRORE: \n")
                print(error)
                K = np.array([-460.5540, -171.0658,  -69.2076,  -26.9682])
                control_input = np.dot(-K, error)  # Apply control law, K is the controller gain matrix
                print("STO STAMPANDO IL TORQUE: \n")
                print(control_input)
                control_torque = np.array([0,control_input])
                pb.setJointMotorControlArray(robotID, jointIndices, controlMode = pb.TORQUE_CONTROL, forces=control_torque)
                # Simulate the dynamics of the acrobot for one time step
                next_state = acrobot_dynamics(q, control_input, 1/240)  # Update the state based on dynamics
                # Update the current state
                q = next_state
            break
        control_torques = swing_up_control(robotModel, q_curr, qdot_curr, qdes_curr, qdotdes_curr)
        print('\n***** control_torques: ', control_torques)
        print('\n')
        print('***** states : ', q_curr, qdot_curr)
        print('\n')
        pb.setJointMotorControlArray(robotID, jointIndices, controlMode = pb.TORQUE_CONTROL, forces=control_torques)

        # compute dynamics
        # nextstate = integration(q_curr, qdot_curr, simDT ,control_torques)
        q_curr, qdot_curr = sim_utils.getState(robotID, jointIndices)
        q_curr = q_curr % (2 * np.pi)
        qdot_curr = qdot_curr % (2 * np.pi)
        pb.stepSimulation()
        time.sleep(simDT)
        # q_kkr = np.array([26.70080267,-16.83162615])
        # for i in jointIndices:
        #     pb.resetJointState(robotID, i, q_kkr[i])

        #input("Press ENTER to continue")


    input("press ENTER to CLOSE the simulation:")

    pb.disconnect()





