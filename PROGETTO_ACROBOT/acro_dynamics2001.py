import numpy as np
import sim_utils
import pybullet as pb
import wrap_utils as wrp



def advance(state, t, k, Δt, ls, ms, lc, Is, g):

    

    θ = np.array(state[:2])
    ω = np.array(state[2:])

    t1 = t[0]
    t2 = t[1]
    t3 = t[2]
    t4 = t[3]
    t5 = t[4]

    kv = k[0]
    ke = k[1]
    kd = k[2]
    kp = k[3]

    q̇ = np.array([ω[0], ω[1] - ω[0]])
    q = np.array([θ[0] - (np.pi / 2), θ[1] - θ[0]])

    cos = np.cos(q[1])
    d11 = t1 + t2 + 2*t3*cos
    d12 = t2 + t3*cos
    d22 = t2
    D = np.array([[d11, d12], [d12, d22]])

    h1 = t3*(-2*q̇[0]*q̇[1] - q̇[1]*q̇[1])*np.sin(q[1])
    h2 = t3*q̇[0]*q̇[0]*np.sin(q[1])
    C = np.array([h1, h2])

    g1 = t4*g*np.cos(q[0]) + t5*g*np.cos(q[0] + q[1])
    g2 = t5*g*np.cos(q[0] + q[1])
    G = np.array([g1, g2])

    Δ = d11*d22 - d12*d12
    E = (0.5*np.dot(q̇,np.dot(D,q̇))) + (t4*g*np.sin(q[0]) + t5*g*np.sin(q[0] + q[1])) # total energy
    Etop = (t4 + t5)*g # Energy at unstable equillibrium, is the energy of Acrobot at the upright equilibrium point
    desired_energy = Etop
    Ẽ = E - Etop
    tau2 = (-(kv*q̇[1] + kp*q[1])*Δ - kd*(d12*(h1 + g1) - d11*(h2 + g2)))/(ke*Ẽ*Δ + kd*d11)
    τ = np.array([0.0, tau2])

    b = τ - C - G

    x = np.linalg.solve(D, b)

    ωdot = np.array([ω[0], ω[1]])
    θdot = np.array([θ[0], θ[1]])

    ωdot[0] += Δt*x[0]
    ωdot[1] += Δt*(x[1] + x[0])
    θdot[0] += Δt*ωdot[0]
    θdot[1] += Δt*ωdot[1]


    qnext = θdot
    q̇next = ωdot

    print('------------------------------------------------------',qnext[0])

    control_torques = τ

    # Total Energy of the Acrobot system
    #acrobot_energy = kinetic_energy_double_pendulum(state, ls, ms, lc, Is, g) + potential_energy_double_pendulum(state, ls, ms, lc, Is, g)

    # Compute the kinetic energy
    M = np.array([[t1+t2+2*t3*np.cos(q[1]), t2+t3*np.cos(q[1])],[t2+t3*np.cos(q[1]), t2]])
    kinetic_energy = 0.5 * np.dot(q̇.T, np.dot(q̇, M))
    

    # Compute the potential energy
    potential_energy = t4*np.sin(q[0]) + t5*np.sin(q[0]+q[1])

    acrobot_energy = kinetic_energy + potential_energy

    #Compute the error between desired energy and current energy
    energy_error = acrobot_energy - desired_energy
    print('error ------------------------------------------------------',Ẽ )

    return qnext, q̇next, control_torques, energy_error

def potential_energy(state, ls, ms, lc, Is, g):
    cms = lc

    term1 = g * (sum([m * cm[1] for (cm, m) in zip(cms, ms)]) + ms[0] * ls[0])
    term2 = g * ms[1] * (ls[1] + 2 * ls[0])
    
    return term1 + term2

def potential_energy_double_pendulum(state, ls, ms, lc, Is, g):
    # Convert angles to radians
    #theta1_rad = np.radians(state[0])
    #theta2_rad = np.radians(state[1])

    # Calculate heights above equilibrium positions
    h1 = ls[0] * (1 - np.cos(state[0]))
    h2 = ls[1] * (1 - np.cos(state[0])) + ls[1] * (1 - np.cos(state[1]))

    # Calculate potential energy using the formula
    potential_energy  = ms[0] * g * h1 + ms[1] * g * h2

    return potential_energy 

def kinetic_energy_double_pendulum(state, ls, ms, lc, Is, g):
    ω = state[-2:]

    kinetic_energy = 0.5 * (Is[0] + ms[0] * ls[0] * ls[0] / 4) * ω[0] * ω[0] + \
         0.5 * (Is[1] + ms[1] * ls[1] * ls[1] / 4) * ω[1] * ω[1] + \
         0.5 * ms[0] * (ls[0] * ls[0] * ω[0] * ω[0] + 4 * np.dot(state[0], state[1]) * ω[0] * ω[1])  # coupling

    return kinetic_energy