import numpy as np
import sim_utils
import pybullet as pb


def calculate_state(parameters, q1, q2, dq1, dq2, torque, dt):
    # Extract the state variables
    q = np.array([q1,q2])
    qdot = np.array([dq1,dq2])
    l1 = parameters[0]
    l2 = parameters[1]
    lc1 = parameters[2]
    lc2 = parameters[3]
    m1 = parameters[4]
    m2 = parameters[5]
    I1 = parameters[6]
    I2 = parameters[7]
    g = 9.81
    alpha1 = m1*(lc1**2) + m2*(l1**2) + I1
    alpha2 = m2*(lc2**2) + I2
    alpha3 = m2*l1*lc2
    beta1 = (m1*lc1 + m2*l1)*g
    beta2 = m2*lc2*g
    tau = torque

    M = np.array([[alpha1 + alpha2 + 2*alpha3*np.cos(q2), alpha2 + alpha3*np.cos(q2)], [alpha2 + alpha3*np.cos(q2), alpha2]])
    M21 = alpha2 + alpha3*np.cos(q2)
    M11 = alpha1 + alpha2 + 2*alpha3*np.cos(q2)
    M22 = alpha2
    BA = np.array([0, 1]).T
    H = alpha3*np.sin(q[1] - q[0])*(np.array([-(qdot[1]**2), qdot[0]**2]).T)
    H1 = alpha3*((-2*dq1*dq2)-(dq2**2))*np.sin(q2)
    H2 = alpha3*(dq1**2)*np.sin(q2)
    C = alpha3*(np.array([((-2*dq1*dq2)-(dq2**2)), dq1**2]).T)*np.sin(q2)
    G = np.array([beta1*np.cos(q1) + beta2*np.cos(q1+q2), beta2*np.cos(q1+q2)]).T
    G1 = beta1*np.cos(q1) + beta2*np.cos(q1+q2)
    G2 = beta2*np.cos(q1+q2)
    qdot = np.array([dq1,dq2])
    E = 0.5*np.dot(np.dot(qdot.T, M),qdot) + beta1*np.sin(q1) + beta2*np.sin(q1+q2)
    delta = alpha1*alpha2 - (alpha3**2)*np.cos(q2)**2
    Er = (beta1+beta2)

    #N = (kv*dq2 + kp*q2)*delta + kd*(M21*(H1+G1) - M11*(H2+G2))
    #D = kd*M11 + (E-Er)*delta

    ddq1 =  - (M21*tau - G2*M21 + G1*M22 - H2*M21 + H1*M22)/(M11*M22 - M21*M21)
    ddq2 = (M11*tau - G2*M11 + G1*M21 - H2*M11 + H1*M21)/(M11*M22 - M21*M21)
    
    # Update the state variables using Euler's method
    next_q1 = q1 + dq1 * dt
    next_q2 = q2 + dq2 * dt
    next_dq1 = dq1 + ddq1 * dt
    next_dq2 = dq2 + ddq2 * dt

    # Return the updated state
    next_state = np.array([next_q1, next_q2, next_dq1, next_dq2])
    return next_state
