import numpy as np
from pinocchio.robot_wrapper import RobotWrapper
import control as control
from control import StateSpace
from control import lqr
from control import ss
""" def stabilization_controller(q, v, desired_position):
    # Define controller gains
    kp = 1.0  # Proportional gain
    kd = 0.5  # Derivative gain

    # Compute the position error
    position_error = desired_position - q[0]

    # Compute the velocity error
    velocity_error = -v[0]

    # Compute control torques
    control_torques = -kp * position_error - kd * velocity_error

    return control_torques """
def stabilization_controller(model,q1,q2,q1_dot,q2_dot):
    position = np.array([q1, -np.pi/2, q2, q1_dot, q2_dot])
    robot = RobotWrapper(model)
    A, B, C, D = robot.linearize()
    sys = ss(A, B, C, D)
    Q = np.eye(4)
    R = 1
    [K,S,P] = lqr(sys,Q,R)
    return K*position

