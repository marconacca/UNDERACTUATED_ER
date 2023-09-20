import numpy as np
from scipy.integrate import odeint

def integration(q, qdot, time, torques):


    # Assuming you have defined the initial state [q0, qdot0]
    initial_state = np.concatenate([q, qdot])
    q0 = q
    qd0 = qdot

    # **********   INTEGRATION with ODEINT   **********
    timesteps = np.linspace(0, time, 10)
    integ_states = odeint(inverse_dynamics, initial_state, timesteps, args=(torques,))
    nextstates = integ_states[-1,:]
    
    # print('states: %s' % states)
    #print('\n ODEint just next_states: %s' % nextstates)
    # input('aaaaaaaaaa')
    
    
    # **********   INTEGRATION with RUNGE-KUTTA Order 4 ("RK4")   **********
    # integ_states = rungekutta4(inverse_dynamics, initial_state, timesteps, args=(torques,))
    # nextstates = integ_states[-1,:]
    
    #print("\n RK4 integration next state: %s" % sol[-1,:])
    #input('aaaaaaaaaaaa')

    
    # **********   EULER INTEGRATION   **********
    #qdd_qd = inverse_dynamics(initial_state, time,torques)
    #qdd = qdd_qd[2:]
    #qdot1 = qd0 + qdd*time
    #q1 = q0 + qdot1*time
    

    # Retrieve the joint positions, velocities
    q1 = nextstates[:2]
    qdot1 = nextstates[2:]
    #print('positions: %s\n, velocities %s\n' % (q1, qdot1))


    return q1, qdot1


def rungekutta4(f, y0, t, args=()):
    n = len(t)
    y = np.zeros((n, len(y0)))
    y[0] = y0
    for i in range(n - 1):
        h = t[i+1] - t[i]
        k1 = f(y[i], t[i], *args)
        k2 = f(y[i] + k1 * h / 2., t[i] + h / 2., *args)
        k3 = f(y[i] + k2 * h / 2., t[i] + h / 2., *args)
        k4 = f(y[i] + k3 * h, t[i] + h, *args)
        y[i+1] = y[i] + (h / 6.) * (k1 + 2*k2 + 2*k3 + k4)
    return y



def inverse_dynamics(state, tstep, torques):

    # #########################   DYNAMICS PARAMETERS and MATRICES   #########################

    # ***** Unpack the state variables *****
    q = np.array(state[:2])
    q1 = q[0]
    q2 = q[1]
    qdot = np.array(state[2:])
    dq1 = qdot[0]
    dq2 = qdot[1]
    #print('q: %s, qdot: %s' % (q, qdot))


    # ***** Paper ARM's PARAMETERS *****
    l1 = 1.
    l2 = 2.
    lc1 = 0.5
    lc2 = 1.
    m1 = 1.
    m2 = 1.
    I1 = 0.083
    I2 = 0.33
    g = 9.8
    # computed torque by our controller
    tau2 = torques[1]
    
    
    # #########################   DYNAMICS PARAMETERS and MATRICES   #########################

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
    G = np.array([beta1*np.cos(q1) + beta2*np.cos(q1+q2), beta2*np.cos(q1+q2)]).T
    G1 = beta1*np.cos(q1) + beta2*np.cos(q1+q2)
    G2 = beta2*np.cos(q1+q2)

    #qdot = np.array([dq1,dq2])
    E = 0.5*np.dot(np.dot(qdot.T, M),qdot) + beta1*np.sin(q1) + beta2*np.sin(q1+q2)
    delta = alpha1*alpha2 - (alpha3**2)*(np.cos(q2)**2)
    Er = (beta1+beta2)
    
    
    # ***************   Acceleration computation   ***************
    ddq1 =  - (M21*tau2 - G2*M21 + G1*M22 - H2*M21 + H1*M22)/(M11*M22 - M21*M21)
    ddq2 = (M11*tau2 - G2*M11 + G1*M21 - H2*M11 + H1*M21)/(M11*M22 - M21*M21)
    # acceleration vector
    qdotdot = np.array([ddq1, ddq2])

    # print('\n qdd1 acceleration: ',ddq1)
    # print(' qdd2 acceleration: ',ddq2)
    # input('stoppa n attimo e vediamo le accelerazioni')
 
 
    
    # Return the derivatives of the state [qdot, qdotdot]
    return np.concatenate((qdot, qdotdot))
    #return qdotdot