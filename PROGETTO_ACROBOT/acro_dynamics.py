import numpy as np
import sim_utils
import pybullet as pb


def acrobot_dynamics(q1,q2,dq1,dq2, control_input, dt):
    # Extract the state variables
    q = np.array([q1,q2])
    qdot = np.array([dq1,dq2])
    
    # Define the acrobot parameters
    # l1 = 0.1425
    # l2 = 0.2305
    # lc1 = 0.035
    # lc2 = 0.1 + l1
    # m1 = 0.26703
    # m2 = 0.33238
    # # inertia for each links (other values are smaller than 10^-8, I keep until 10^-5)
    # I1 = 0.000036421
    # I2 = 0.000014553

    l1 = 1
    l2 = 2
    lc1 = 0.5
    lc2 = 1
    m1 = 1
    m2 = 1
    # inertia for each links (other values are smaller than 10^-8, I keep until 10^-5)
    I1 = 0.083
    I2 = 0.33
    g = 9.81
    tau = control_input

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
    qdot = np.array([dq1,dq2])
    E = 0.5*np.dot(np.dot(qdot.T, M),qdot) + beta1*np.sin(q1) + beta2*np.sin(q1+q2)
    delta = alpha1*alpha2 - (alpha3**2)*np.cos(q2)**2
    #E = 0.5*np.dot(qdot.T,np.dot(M,qdot)) + beta1*(np.cos(q[0]) - 1) + beta2*(np.cos(q[1]) - 1)
    # Er = -2*(beta1+beta2)
    Er = (beta1+beta2)

    # N = (kv*dq2 + kp*q2)*delta + kd*(M21*(H1+G1) - M11*(H2+G2))
    # D = kd*M11 + (E-Er)*delta
    # d11 = m1 * lc1**2 + m2*(l1**2 + lc2**2 + 2*l1*lc2)*np.cos(q[1]) + I1 + I2
    # d12 = m2*(lc2**2 + l1*lc2*np.cos(q[1])) + I2
    # d22 = m2 * lc2**2 + I2

    # c1 = -m2*l1*lc2*np.sin(q[1])*qdot[1]**2 - 2*m2*l1*lc2*np.sin(q[1])*qdot[0]*qdot[1]
    # c2 = m2*l1*lc2*np.sin(q[1])*qdot[0]**2

    # phi1 = (m1*lc1**2 + m2*l1)*g*np.cos(q[0]) + m2*lc2*g*np.cos(q[0]+ q[1])
    # phi2 = m2*lc2*g*np.cos(q[0]+ q[1])

    # Compute the joint accelerations using the equations of motion
    # ddq2 = (d11*(tau - c2 -phi2) + d12*(c1+phi1))/(d11*d22 - d12**2)
    # ddq1 = (d12*ddq2 + c1 + phi1)/(-d11)

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
