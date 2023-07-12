import numpy as np
from scipy.integrate import odeint

def integration(q, qdot, time, torques):
    # Define the equations of motion based on your robot dynamics
    # state: array of joint positions and velocities [q, qdot]
    # t: time variable (unused if dynamics are time-invariant)
    # Return the derivatives of the state [qdot, qdotdot]
    # Example: return np.array([qdot, qdotdot])

    # Assuming you have defined the initial state [q0, qdot0]
    initial_state = np.concatenate([q, qdot])
    q0 = q
    qd0 = qdot


    # Assuming you have defined the input torque array over time
    # timesteps: array of time values
    # torques: array of torques corresponding to each time step

    # Perform numerical integration using odeint
    timesteps = np.linspace(0, time, 10)
    #print('timesteps: %s' % timesteps)
    states = odeint(dynamics, initial_state, timesteps, args=(torques,))
    #print('!!!!!!!!!!!!! States: ', states)
    

    # qdd = dynamics(initial_state, time,torques)
    # # Euler integration
    # qdot1 = qd0 + qdd*time
    # q1 = q0 + qdot1*time

    # Retrieve the joint positions, velocities, and accelerations
    positions = states[:, :2]  # n: number of joints
    velocities = states[:, 2:]
    #print('positions: %s\n, velocities %s\n' % (positions, velocities))
    #input('dajedajedajed')
    #print(positions, velocities)

    return positions[-1,:], velocities[-1,:]
    #return q1, qdot1




def dynamics(state, tstep, torques):
    # Unpack the state variables

    # Define the parameters of your robot (e.g., mass, inertia)
    # and any other necessary constants

    q = np.array(state[:2])
    qdot = np.array(state[2:])

    #print('q: %s, qdot: %s' % (q, qdot))

    l1 = 0.1425
    l2 = 0.2305
    lc1 = 0.035
    lc2 = 0.1
    m1 = 0.26703
    m2 = 0.33238
    # inertia for each links (other values are smaller than 10^-8, I keep until 10^-5)
    I1 = np.matrix([[0.00040827, 0, 0.000018738], [0, 0.00038791, 0], [0.000018738, 0, 0.000036421]])
    I2 = np.matrix([[0.0011753, 0, 0], [0, 0.0011666, 0], [0, 0, 0.000014553]])
    Iz1 = I1[2,2]
    Iz2 = I2[2,2]
    g = 9.81

    d11 = m1 * lc1**2 + m2*(l1**2 + lc2**2 + 2*l1*lc2)*np.cos(q[1]) + Iz1 + Iz1
    d12 = m2*(lc2**2 + l1*lc2*np.cos(q[1])) + Iz2
    d22 = m2 * lc2**2 + Iz2

    c1 = -m2*l1*lc2*np.sin(q[1])*qdot[1]**2 - 2*m2*l1*lc2*np.sin(q[1])*qdot[0]*qdot[1]
    c2 = m2*l1*lc2*np.sin(q[1])*qdot[0]**2

    phi1 = (m1*lc1**2 + m2*l1)*g*np.cos(q[0]) + m2*lc2*g*np.cos(q[0]+ q[1])
    phi2 = m2*lc2*g*np.cos(q[0]+ q[1])

    # computed torque by our controller
    #torque = torques
    tau2 = torques[1]

    # Compute the joint accelerations using the equations of motion
    q2_dot_dot = (d11*(tau2 - c2 -phi2) + d12*(c1+phi1))/(d11*d22 - d12**2)
    q1_dot_dot = (d12*q2_dot_dot + c1 + phi1)/(-d11)

    qdotdot = np.array([q1_dot_dot, q2_dot_dot])

    # print(' qdd1 acceleration: ',q1_dot_dot)
    # print(' qdd2 acceleration: ',q2_dot_dot)
    # input('stoppa n attimo e vediamo le accelerazioni')
    
    
    

    
    # Return the derivatives of the state [qdot, qdotdot]
    return np.concatenate([qdot, qdotdot])
    #return qdotdot