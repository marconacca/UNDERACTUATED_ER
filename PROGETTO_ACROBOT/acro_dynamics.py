import numpy as np
import control
import sim_utils
import pybullet as pb


def acrobot_dynamics(state, control_input, dt):
    # Extract the state variables
    q1, q2, dq1, dq2 = state
    q = np.array([q1,q2])
    qdot = np.array([dq1,dq2])
    # Define the acrobot parameters
    l1 = 0.1425
    l2 = 0.2305
    lc1 = 0.035
    lc2 = 0.1 + l1
    m1 = 0.26703
    m2 = 0.33238
    # inertia for each links (other values are smaller than 10^-8, I keep until 10^-5)
    I1 = 0.000036421
    I2 = 0.000014553
    g = 9.81
    tau = control_input
    # Compute the derivatives of the state variables
    # Equation for the derivative of q1
    # ddq1 = (m2 * l1 * lc2 * np.sin(q2) * (dq2**2) + m2 * g * lc2 * np.sin(q2) * np.cos(q2) +
    #         m2 * l2 * lc2 * (dq2**2) * np.sin(q2) + (I1 + I2 + m2 * l1**2 + m2 * l2**2) * tau -
    #         2 * m2 * l1 * lc2 * dq1 * dq2 * np.sin(q2)) / (I1 * I2 + I1 * m2 * l1**2 + I2 * m2 * l2**2 +
    #                                                     m2**2 * l1**2 * l2**2 - (m2 * l1 * lc2)**2 * np.cos(q2)**2)

    # # Equation for the derivative of q2
    # ddq2 = (-m2 * l1 * lc2 * np.sin(q2) * (dq1**2) - m2 * g * lc2 * np.sin(q2) -
    #         m2 * l1 * lc2 * (dq1**2) * np.sin(q2) - (I1 + m2 * l1**2) * tau * np.cos(q2) +
    #         m2 * l1 * lc2 * dq1 * dq2 * np.sin(q2)) / (I1 * I2 + I1 * m2 * l1**2 + I2 * m2 * l2**2 +
    #                                                 m2**2 * l1**2 * l2**2 - (m2 * l1 * lc2)**2 * np.cos(q2)**2)

    d11 = m1 * lc1**2 + m2*(l1**2 + lc2**2 + 2*l1*lc2)*np.cos(q[1]) + I1 + I2
    d12 = m2*(lc2**2 + l1*lc2*np.cos(q[1])) + I2
    d22 = m2 * lc2**2 + I2

    c1 = -m2*l1*lc2*np.sin(q[1])*qdot[1]**2 - 2*m2*l1*lc2*np.sin(q[1])*qdot[0]*qdot[1]
    c2 = m2*l1*lc2*np.sin(q[1])*qdot[0]**2

    phi1 = (m1*lc1**2 + m2*l1)*g*np.cos(q[0]) + m2*lc2*g*np.cos(q[0]+ q[1])
    phi2 = m2*lc2*g*np.cos(q[0]+ q[1])

    # Compute the joint accelerations using the equations of motion
    ddq2 = (d11*(tau - c2 -phi2) + d12*(c1+phi1))/(d11*d22 - d12**2)
    ddq1 = (d12*ddq2 + c1 + phi1)/(-d11)

    # Update the state variables using Euler's method
    next_q1 = q1 + dq1 * dt
    next_q2 = q2 + dq2 * dt
    next_dq1 = dq1 + ddq1 * dt
    next_dq2 = dq2 + ddq2 * dt

    # Return the updated state
    next_state = np.array([next_q1, next_q2, next_dq1, next_dq2])
    return next_state
