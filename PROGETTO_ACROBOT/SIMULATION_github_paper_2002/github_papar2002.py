import numpy as np

def energy_swingup_demo(state, ts, g, Δt):
    
    τ, Ẽ, D, C, G = energy_swingup(state, ts, g)

    qnext, q̇next = advance(state, D, C, G, Δt, τ)


    return qnext, q̇next, τ, Ẽ



def energy_swingup(state, t, g):
    
    kv = 45.0
    ke = 0.5
    kd = 15.0
    kp = 22.0

    t1 = t[0]
    t2 = t[1]
    t3 = t[2]
    t4 = t[3]
    t5 = t[4]

    θ = np.array(state[:2])
    ω = np.array(state[2:])

    q̇ = np.array([ω[0], ω[1] - ω[0]])
    q = np.array([θ[0] - np.pi/2, θ[1] - θ[0]])

    q1 = q[0]
    q2 = q[1]
    q̇1 = q̇[0]
    q̇2 = q̇[1]

    # Compute dynamic matrices
    d11 = t1 + t2 + 2*t3*np.cos(q2)
    d12 = t2 + t3*np.cos(q2)
    d22 = t2
    D = np.array([[d11, d12], [d12, d22]])

    # Compute Coriolis and centrifugal forces vector
    h1 = t3*(-2*q̇1*q̇2 - q̇1*q̇1)*np.sin(q2)
    h2 = t3*q̇1*q̇1*np.sin(q2)
    C = np.array([h1, h2])

    # Compute gravitational forces vector
    g1 = t4*g*np.cos(q1) + t5*g*np.cos(q1 + q2)
    g2 = t5*g*np.cos(q1 + q2)
    G = np.array([g1, g2])

    # Compute energy-related variables
    Δ = d11*d22 - d12*d12
    n = (0.5*np.dot(q̇, np.dot(D, q̇))) 
    n1 = t4*g*np.sin(q1)
    n2 = (t5*g*np.sin(q1 + q2)) 
    E =  n + n1 + n2  # total energy
    
    Etop = (t4 + t5)*g  # Energy at unstable equilibrium
    Ẽ = E - Etop

    # Compute control input (torque)
    control_input = (-(kv*q̇2 + kp*q2)*Δ - kd*(d12*(h1 + g1) - d11*(h2 + g2)))/(ke*Ẽ*Δ + kd*d11)

    τ = np.array([0.0, control_input])

    print('torque--------------------------',τ)
    print('energy error--------------------------',Ẽ)
    

    return τ, Ẽ, D, C, G


def advance(state, D, C, G, Δt, τ):
    
    # Compute b = τ - C - G
    b = τ - C - G

    # Solve the system Dx = b
    x = np.linalg.solve(D, b)

    # Update angular positions and velocities
    ωdot = np.array([state[2], state[3]])
    θdot = np.array([state[0], state[1]])

    ωdot[0] += Δt * x[0]
    ωdot[1] += Δt * (x[1] + x[0])
    θdot[0] += Δt * ωdot[0]
    θdot[1] += Δt * ωdot[1]

    qnext = θdot
    q̇next = ωdot

    print('q1------------------------------------------------------',qnext[0])
    print('q2------------------------------------------------------',qnext[1])

    return qnext, q̇next

def switch(q, qdot):

    zeta = 0.04 #threshold to switch controller (Energy to LQR)
    x_state = np.array([q[0]-np.pi/2, q[1], qdot[0], qdot[1]])
    condition = (abs(x_state[0]) + abs(x_state[1]) + 0.1*(abs(x_state[2])) + 0.1*(abs(x_state[3])))
    if (condition < zeta):
        return True
    
    return False