import numpy as np
import control as control
import pybullet as pb
from control import StateSpace
from control import lqr
from control import ss
import os
import pinocchio as pin
from scipy.linalg import solve_continuous_are
import sim_utils

def stabilization_controller(model,q1,q2,q1_dot,q2_dot,data,robotID,desired_position):
    num_joints = pb.getNumJoints(robotID)
    print(num_joints)
    joint_indices = range(num_joints)
    # Create the system matrices
    A = np.zeros((num_joints, num_joints))
    print(A)
    B = np.zeros((num_joints, 1))
    Q = np.diag([1.0] * num_joints)
    R = np.array([[1.0]])
    position = np.array([q1, np.pi/2, q1_dot, q2_dot])
    desired_pos = np.array([desired_position[0], desired_position[1], q1_dot, q2_dot])
    #position = np.array([np.pi/2, -np.pi/2, 0, 0])
    # Compute the LQR gain
    # K, S, E = lqr(A, B, Q, R)
    # P = solve_continuous_are(A, B, Q, R)
    # Compute the LQR gain
    # K = np.linalg.inv(R) @ B.T @ P
    F = np.array([-246.481,-98.690,-106.464,-50.138])
    return -F@(position-desired_pos)