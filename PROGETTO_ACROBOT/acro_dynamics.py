import numpy as np
import sim_utils
import pybullet as pb
import pinocchio as pin


def acrobot_dynamics(q1,q2,dq1,dq2, control_input, dt, robotModel):
    # Extract the state variables
    q = np.array([q1-np.pi/2,q2-q1])
    qdot = np.array([dq1,dq2-dq1])
    
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
    g = 9.8
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
    H1 = alpha3*np.sin(q2)*((-2*dq1*dq2)-(dq2**2))
    H2 = alpha3*np.sin(q2)*(dq1**2)
    C = np.array([[H1], [H2]])
    G1 = beta1*np.cos(q1) + beta2*np.cos(q1+q2)
    G2 = beta2*np.cos(q1+q2)
    G = np.array([[G1], [G2]])
    b = tau - C - G
    x = np.linalg.solve(M, b)
    print("Sto printando x: \n",np.round(x, decimals = 2))
    next_dq1 = dq1 + dt*x[0]
    next_dq2 = dq2 + dt*x[1]
    next_q1 = q1 + dt*next_dq1
    next_q2 = q2 + dt*next_dq2
    
    # Return the updated state
    next_state = np.array([next_q1, next_q2, next_dq1, next_dq2])
    return next_state
