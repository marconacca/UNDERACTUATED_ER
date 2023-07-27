import numpy as np
import sim_utils
import pybullet as pb
from wrap_utils import *



def advance(state, θ, k, Δt, ls, ms, lc, Is, g, A, B):

    q = np.array(state[:2])
    q̇ = np.array(state[2:])

    zeta = 0.04
    state_paper = np.array([q[0]-np.pi/2, q[1], q̇[0], q̇[1]])
    condition = abs(state_paper[0]) + abs(state_paper[1]) + abs(state_paper[2])*0.1 + abs(state_paper[3])*0.1

    θ1 = θ[0]
    θ2 = θ[1]
    θ3 = θ[2]
    θ4 = θ[3]
    θ5 = θ[4]

    kv = k[0]
    ke = k[1]
    kd = k[2]
    kp = k[3]

    q̇ = np.array([q̇[0], q̇[1] - q̇[0]])
    q = np.array([q[0] - (np.pi / 2), q[1] - q[0]])
    q = normalize_angles_2002(q)
    #q = np.array([q[0] - (np.pi / 2), q[1]])
    

    q1 = q[0]
    q2 = q[1]
    q̇1 = q̇[0]
    q̇2 = q̇[1]

    #q̇ = np.array([ω[0], ω[1] - ω[0]])
    #q = np.array([θ[0] - (np.pi / 2), θ[1] - θ[0]])


    d11 = θ1 + θ2 + 2*θ3*np.cos(q2)
    d12 = θ2 + θ3*np.cos(q2)
    d22 = θ2
    D = np.array([[d11, d12], [d12, d22]])

    h1 = θ3*(-2*q̇1*q̇2 - q̇1**2)*np.sin(q2)
    h2 = θ3*(q̇1**2)*np.sin(q2)
    C = np.array([h1, h2])

    g1 = θ4*g*np.cos(q1) + θ5*g*np.cos(q1 + q2)
    g2 = θ5*g*np.cos(q1 + q2)
    G = np.array([g1, g2])

    Δ = d11*d22 - d12**2
    E = (0.5*np.dot(q̇,np.dot(D,q̇))) + (θ4*g*np.sin(q1) + θ5*g*np.sin(q1 + q2)) # total energy
    Etop = (θ4 + θ5)*g # Energy at unstable equillibrium, is the energy of Acrobot at the upright equilibrium point
    Ẽ = E - Etop
    tau2 = (-(kv*q̇2 + kp*q2)*Δ - kd*(d12*(h1 + g1) - d11*(h2 + g2)))/(ke*Ẽ*Δ + kd*d11)
    τ = np.array([0.0, tau2])

    b = τ - C - G

    x = np.linalg.solve(D, b)

    """

    ddq1 =  - (d12*tau2 - g2*d12 + g1*d22 - h2*d12 + h1*d22)/(d11*d22 - d12**2)
    ddq2 = (d11*tau2 - g2*d11 + g1*d12 - h2*d11 + h1*d12)/(d11*d22 - d12**2)

    next_dq1 = q̇1 + ddq1 * Δt
    next_dq2 = q̇2 + ddq2 * Δt
    next_q1 = q1 + next_dq1 * Δt
    next_q2 = q2 + next_dq2 * Δt

    qnext = np.array([next_q1,next_q2])
    qnext = normalize_angles_2002(qnext)
    q̇next = np.array([next_dq1,next_dq2])

    """

    ωdot = np.array([state[2], state[3]])
    θdot = np.array([state[0], state[1]])

    #ωdot = np.array([q̇1, q̇2])
    #θdot = np.array([q1, q2])

    ωdot[0] += Δt*x[0]
    ωdot[1] += Δt*(x[1]+ x[0])
    θdot[0] += Δt*ωdot[0]
    θdot[1] += Δt*ωdot[1]

    qnext = θdot
    qnext = normalize_angles_2002(qnext)
    q̇next = ωdot
    

    


    

    

    print('q1------------------------------------------------------',qnext[0])
    print('q2------------------------------------------------------',qnext[1])

    control_torques = τ


    #//////////////////////////////Questo codice si basa sulle funzioni tradotte da github/////////////////////
    #fa più o meno la stessa cosa del codice dentro la funzione swing_up() di github, non ha senso usarlo

    # Total Energy of the Acrobot system
    #acrobot_energy = kinetic_energy_double_pendulum(state, ls, ms, lc, Is, g) + potential_energy_double_pendulum(state, ls, ms, lc, Is, g)
    #desired_energy = Etop
    #energy_error = acrobot_energy - desired_energy
    #print('code energy error github function ------------------------------------------------------   ',energy_error)

    #//////////////////////////////Questo codice ha lo stesso output del codice preso da github/////////////////////
    #se però si rimuove il cambio di coordinate l'energy error differisce, ma si blocca allo stesso modo di quello 
    # ottenuto dal codice github (github = -49,525 mentre questo = -27 entrambi fissi)


    # Compute the kinetic energy
    #M = np.array([[θ1+θ2+2*θ3*np.cos(q2), θ2+θ3*np.cos(q2)],[θ2+θ3*np.cos(q2), θ2]])
    #kinetic_energy = 0.5 * np.dot(q̇.T, np.dot(q̇, M))
    

    # Compute the potential energy
    #potential_energy = θ4*np.sin(q1) + θ5*np.sin(q1+q2)

    #acrobot_energy = kinetic_energy + potential_energy

    #Compute the error between desired energy and current energy
    #desired_energy = Etop
    #energy_error = acrobot_energy - desired_energy
    #print('code energy error 2007 ------------------------------------------------------   ',energy_error)


    #/////////////////////////////////////////////////////////////////////////////////////////////////////////////

    energy_error = Ẽ

    
    print('code energy error ------------------------------------------------------   ',energy_error)
    

    if condition < zeta:
        print("-------------------------------------------------------------------------------------LQR-CONTROL:")
        

    #qnext, q̇next = advance_2002(state, ms, Is, kd, g, Δt)
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



def advance_2002(state, ms, Is, kd, g, Δt):

    M = np.array([[ms[0], 0],
              [0, ms[1]]])
    I = np.array([[Is[0], 0],
              [0, Is[1]]])
    
    # Construct the matrix A
    A = create_A(M, I, state[:2])

    # Define the vector b
    b = np.zeros(6)

    # Fill the values of b
    ω = np.array(state[2:])
    component01 = ms * np.array([0.0, -g])
    component23 = -kd * ω
    component45 = -ω * ω * np.array(state[:2])
    b = np.array([component01[0], component01[1], component23[0], component23[1], component45[0], component45[1]])

    x = np.linalg.solve(A, b)  # solve system

    θ_prime = np.zeros(2)
    ω_prime = np.zeros(2)
    stateω = np.array(state[2:]) 
    stateθ = np.array(state[:2])

    # Update configuration
    for i in range(2):
        ω_prime[i] = stateω[i] + x[2 + 5*i] * Δt
        θ_prime_tmp = stateθ[i] + ω_prime[i] * Δt
        θ_prime[i] = θ_prime_tmp + (2 * np.pi if θ_prime_tmp <= 0 else (-2 * np.pi if θ_prime_tmp > 2 * np.pi else 0.0))  # normalize angles

    return θ_prime, ω_prime




def create_A(M, I, r):
    # Calculate the skew-symmetric matrix r̃
    r̃ = np.array([[0, -r[1]],
                   [r[0], 0]])

    # Get the size of the matrix
    N = M.shape[0]

    # Construct the matrix A as a zero matrix of size N*3 x N*3
    A = np.zeros((N*3, N*3))

    # Fill the values of A
    A[0:N, 0:N] = M
    A[0:N, N:2*N] = np.zeros((N, N))
    A[N:2*N, 0:N] = np.zeros((N, N))
    A[N:2*N, N:2*N] = I
    A[N:2*N, 2*N:3*N] = r̃
    A[2*N:3*N, 0:N] = -np.eye(N)
    A[2*N:3*N, N:2*N] = r̃
    A[2*N:3*N, 2*N:3*N] = np.zeros((N, N))

    return A