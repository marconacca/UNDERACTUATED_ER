import numpy as np
import sim_utils
import pybullet as pb


def acrobot_dynamics(q, qdot, control_input, dt):
    # state components

    #q1 = q[0] 
    #q2 = q[1]
    #dq1 = qdot[0]
    #dq2 = qdot[1]

    q1 = (q[0] - np.pi/2) 
    q2 = (q[1] - q[0])
    dq1 = qdot[0]
    dq2 = (qdot[1] - qdot[0])

    #removed wrapping, wrap_utils not pushed for compiling code

    l1 = 1
    l2 = 2
    lc1 = 0.5
    lc2 = 1
    m1 = 1
    m2 = 1
    I1 = 0.083
    I2 = 0.33
    g = 9.81
    torques = control_input
    #tau2 = control_input[1]

    alpha1 = m1*(lc1**2) + m2*(l1**2) + I1
    alpha2 = m2*(lc2**2) + I2
    alpha3 = m2*l1*lc2
    beta1 = (m1*lc1 + m2*l1)*g
    beta2 = m2*lc2*g

    M = np.array([[alpha1 + alpha2 + 2*alpha3*np.cos(q2), alpha2 + alpha3*np.cos(q2)], [alpha2 + alpha3*np.cos(q2), alpha2]])
    M21 = alpha2 + alpha3*np.cos(q2)
    M11 = alpha1 + alpha2 + 2*alpha3*np.cos(q2)
    M22 = alpha2
    BA = np.array([0, 1]).T
    H = alpha3*np.sin(q[1] - q[0])*(np.array([-(qdot[1]**2), qdot[0]**2]).T)
    H1 = alpha3*np.sin(q2)*((-2*dq1*dq2)-(dq2**2))
    H2 = alpha3*np.sin(q2)*(dq1**2)
    C = alpha3*(np.array([-2*dq1*dq2 - dq2**2, dq1**2]).T)*np.sin(q2)
    # G = np.array([-beta1*np.sin(q[0]), -beta2*np.sin(q[1])]).T
    G = np.array([beta1*np.cos(q1) + beta2*np.cos(q1+q2), beta2*np.cos(q1+q2)]).T
    G1 = beta1*np.cos(q1) + beta2*np.cos(q1+q2)
    G2 = beta2*np.cos(q1+q2)
    #E = 0.5*np.dot(np.dot(qdot.T, M),qdot) + beta1*np.sin(q1) + beta2*np.sin(q1+q2)
    #delta = alpha1*alpha2 - (alpha3**2)*(np.cos(q2)**2)
    #Er = (beta1+beta2)

    # Update the state with Integration
    b = torques - C - G
    x = np.linalg.solve(M, b)
    
    """
    ddq1 =  - (M21*tau2 - G2*M21 + G1*M22 - H2*M21 + H1*M22)/(M11*M22 - M21*M21)
    ddq2 = (M11*tau2 - G2*M11 + G1*M21 - H2*M11 + H1*M21)/(M11*M22 - M21*M21)
    next_dq1 = dq1 + np.int64(x[0]) * dt
    next_dq2 = dq2 + np.int64((x[0] + x[1])) * dt
    next_q1 = q1 + next_dq1 * dt
    next_q2 = q2 + next_dq2 * dt
    """

    # Update the state variables using Euler's method
    # next_dq1 = dq1 + ddq1 * dt
    # next_dq2 = dq2 + ddq2 * dt
    # next_q1 = q1 + next_dq1 * dt
    # next_q2 = q2 + next_dq2 * dt

    # Return the updated state
    #next_state = np.array([next_q1, next_q2, next_dq1, next_dq2])
    #return next_state

    next_q1 = q1 + dt*dq1
    next_q2 = q2 + dt*dq2
    next_dq1 = dq1 + dt*(x[0])
    next_dq2 = dq2 + dt*(x[1])
    
    return [next_q1, next_q2], [next_dq1, next_dq2]
