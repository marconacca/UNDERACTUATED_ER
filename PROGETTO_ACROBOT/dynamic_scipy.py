import numpy as np
from scipy.integrate import odeint

def integration(q, qdot, time, torques):


    # Assuming you have defined the initial state [q0, qdot0]
    initial_state = np.concatenate([q, qdot])
    q0 = q
    qd0 = qdot

    # **********   INTEGRATION with ODEINT   **********
    timesteps = np.linspace(0, time, 2)
    #print('timesteps: %s' % timesteps)
    states = odeint(dynamics, initial_state, timesteps, args=(torques,))
    nextstates = states[-1,:]
    #print('states: %s' % states)
    #input('aaaaaaaaaa')
    

    #qdd = dynamics(initial_state, time,torques)
    # **********   EULER INTEGRATION   **********
    #qdot1 = qd0 + qdd*time
    #q1 = q0 + qdot1*time

    # Retrieve the joint positions, velocities
    q1 = nextstates[:2]
    qdot1 = nextstates[2:]
    #print('positions: %s\n, velocities %s\n' % (q1, qdot1))


    return q1, qdot1




def dynamics(state, tstep, torques):
    # Unpack the state variables

    # Define the parameters of your robot (e.g., mass, inertia) and any other necessary constants

    q = np.array(state[:2])
    q1 = q[0]
    q2 = q[1]
    qdot = np.array(state[2:])
    dq1 = qdot[0]
    dq2 = qdot[1]
    #print('q : ',q)
    #print('qdot : ',qdot)

    #print('q: %s, qdot: %s' % (q, qdot))

    # # l1 = 0.1425
    # # l2 = 0.2305
    # # lc1 = 0.035
    # # lc2 = 0.1
    # l1 = 0.102611
    # l2 = 0.220227
    # lc1 = 0.0370271
    # lc2 = 0.101004
    # m1 = 0.26703
    # m2 = 0.33238
    # # inertia for each links (other values are smaller than 10^-8, I keep until 10^-5)
    # I1 = np.matrix([[0.00040827, 0, 0.000018738], [0, 0.00038791, 0], [0.000018738, 0, 0.000036421]])
    # I2 = np.matrix([[0.0011753, 0, 0], [0, 0.0011666, 0], [0, 0, 0.000014553]])
    # #I1 = I1[0,0]  # x-axis inertia
    # #I2 = I2[0,0]
    # I1 = I1[2,2]  # z-axis inertia
    # I2 = I2[2,2]
    # g = 9.81
    # # computed torque by our controller
    # tau2 = torques[1]
    

    # Paper parameters
    l1 = 1
    l2 = 2
    lc1 = 0.5
    lc2 = 1
    m1 = 1
    m2 = 1
    I1 = 0.083
    I2 = 0.33
    g = 9.8
    # computed torque by our controller
    tau2 = torques[1]
    

    d11 = m1 * lc1**2 + m2*(l1**2 + lc2**2 + 2*l1*lc2)*np.cos(q[1]) + I1 + I2
    d12 = m2*(lc2**2 + l1*lc2*np.cos(q[1])) + I2
    d22 = m2 * lc2**2 + I2

    c1 = -m2*l1*lc2*np.sin(q[1])*qdot[1]**2 - 2*m2*l1*lc2*np.sin(q[1])*qdot[0]*qdot[1]
    c2 = m2*l1*lc2*np.sin(q[1])*qdot[0]**2

    phi1 = (m1*lc1**2 + m2*l1)*g*np.cos(q[0]) + m2*lc2*g*np.cos(q[0]+ q[1])
    phi2 = m2*lc2*g*np.cos(q[0]+ q[1])


    # Compute the joint accelerations using the equations of motion
    ddq2 = (d11*(tau2 - c2 -phi2) + d12*(c1+phi1))/(d11*d22 - d12**2)
    ddq1 = (d12*ddq2 + c1 + phi1)/(-d11)



    # alpha1 = m1*(lc1**2) + m2*(l1**2) + I1
    # alpha2 = m2*(lc2**2) + I2
    # alpha3 = m2*l1*lc2
    # beta1 = (m1*lc1 + m2*l1)*g
    # beta2 = m2*lc2*g

    # M = np.array([[alpha1 + alpha2 + 2*alpha3*np.cos(q2), alpha2 + alpha3*np.cos(q2)], [alpha2 + alpha3*np.cos(q2), alpha2]])
    # M21 = alpha2 + alpha3*np.cos(q2)
    # M11 = alpha1 + alpha2 + 2*alpha3*np.cos(q2)
    # M22 = alpha2
    # BA = np.array([0, 1]).T
    # H = alpha3*np.sin(q[1] - q[0])*(np.array([-(qdot[1]**2), qdot[0]**2]).T)
    # H1 = alpha3*np.sin(q2)*((-2*dq1*dq2)-(dq2**2))
    # H2 = alpha3*np.sin(q2)*(dq1**2)
    # C = alpha3*(np.array([-2*dq1*dq2 - dq2**2, dq1**2]).T)*np.sin(q2)
    # # G = np.array([-beta1*np.sin(q[0]), -beta2*np.sin(q[1])]).T
    # G = np.array([beta1*np.cos(q1) + beta2*np.cos(q1+q2), beta2*np.cos(q1+q2)]).T
    # G1 = beta1*np.cos(q1) + beta2*np.cos(q1+q2)
    # G2 = beta2*np.cos(q1+q2)
    # qdot = np.array([dq1,dq2])
    # E = 0.5*np.dot(np.dot(qdot.T, M),qdot) + beta1*np.sin(q1) + beta2*np.sin(q1+q2)
    # delta = alpha1*alpha2 - (alpha3**2)*(np.cos(q2)**2)
    # #E = 0.5*np.dot(qdot.T,np.dot(M,qdot)) + beta1*(np.cos(q[0]) - 1) + beta2*(np.cos(q[1]) - 1)
    # # Er = -2*(beta1+beta2)
    # Er = (beta1+beta2)

    # ddq1 =  - (M21*tau2 - G2*M21 + G1*M22 - H2*M21 + H1*M22)/(M11*M22 - M21*M21)
    # ddq2 = (M11*tau2 - G2*M11 + G1*M21 - H2*M11 + H1*M21)/(M11*M22 - M21*M21)


    # acceleration vector
    qdotdot = np.array([ddq1, ddq2])

    #print(' qdd1 acceleration: ',ddq1)
    #print(' qdd2 acceleration: ',ddq2)
    #input('stoppa n attimo e vediamo le accelerazioni')

    
    # Return the derivatives of the state [qdot, qdotdot]
    return np.concatenate([qdot, qdotdot])
    #return qdotdot