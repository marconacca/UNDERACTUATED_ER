import numpy as np


def energy_swingup_demo(q, q̇):
    
    #sim = Simulation{2}(swing_bot; Δt=0.02, kd=0.0)
    
    l1 = 1
    l2 = 2
    ls = np.array([l1, l2])
    lc1 = 0.5
    lc2 = 1
    lc = np.array([lc1, lc2])
    m1 = 1
    m2 = 1
    ms = np.array([m1, m2])
    I1 = 0.083
    I2 = 0.33
    Is = np.array([I1, I2])
    g = 9.8
    
    Δt=0.02
    
    
    t1 = ms[0]*lc[0]*lc[0] + ms[1]*ls[0]*ls[0] + Is[0]
    t2 = ms[1]*lc[1]*lc[1] + Is[1]
    t3 = ms[1]*ls[0]*lc[1]
    t4 = ms[0]*lc[0] + ms[1]*ls[0]
    t5 = ms[1]*lc[1]
    

    # Energy based swing-up torque

    kv = 45.0
    ke = 0.5
    kd = 15.0
    kp = 22.0
    q = np.array([q[0]-np.pi/2, q[1] - q[0]])
    q̇ = np.array([q̇[0], q̇[1] - q̇[0]])

    d11 = t1 + t2 + 2*t3*np.cos(q[1])
    d12 = t2 + t3*np.cos(q[1])
    d22 = t2
    D = np.array([[d11, d12], [d12, d22]])

    h1 = t3*(-2*q̇[0]*q̇[1] - q̇[1]*q̇[1])*np.sin(q[1])
    h2 = t3*q̇[0]*q̇[0]*np.sin(q[1])
    C = np.array([h1, h2])

    g1 = t4*g*np.cos(q[0]) + t5*g*np.cos(q[0] + q[1])
    g2 = t5*g*np.cos(q[0] + q[1])
    G = np.array([g1, g2])

    Δ = d11*d22 - d12*d12
    E = 0.5*q̇.T*D*q̇ + t4*g*np.sin(q[0]) + t5*g*np.sin(q[0] + q[1]) # total energy
    Etop = (t4 + t5)*g # Energy at unstable equillibrium
    Ẽ = E - Etop
    tau2 = (-(kv*q̇[1] + kp*q[1])*Δ - kd*(d12*(h1 + g1) - d11*(h2 + g2)))/(ke*Ẽ*Δ + kd*d11)
    # print('shape:', np.shape(tau2))
    # print(tau2)
    τ = np.array([0.0, tau2[0,0]])
        
    
    # Alternative 2-link pendulum simulator from
    # "The Swing up Control for the Acrobot based on Energy Control Approach" by Xin and Kaneda
    # This simulator supports only two links and computes only angular acceleration.
    #=function advance(state::State{2}, sim::Simulation{2}, τ::SVector{2,Float64})
    
    
    b = τ - C - G

    #x = b*np.linalg.inv(D)
    x = np.linalg.solve(D, b)
    
    print(' Tau :', τ)
    print(' D :', D)
    print(' C :', C)
    print(' G :', G)
    print(' b: ', b)
    print('x:', x)
    print('shape x :', x.shape)
    
    qnext = q
    q̇next = q̇

    q̇next[0] += Δt*x[0]
    q̇next[1] += Δt*(x[1] + x[0])
    q[0] += Δt*q̇next[0]
    q[1] += Δt*q̇next[1]
    state = (qnext, q̇next)
    
    #anim(init_horizontal(Val{2}), sim, energy_swingup, advance, "energy_swingup_test", 2000)
    
    return qnext, q̇next
    


q0 = np.array([0, 0])
q̇0 = np.array([0, 0])

for i in range(1000):
    
    qnext, q̇next = energy_swingup_demo(q0,q̇0)
    
    q0 = qnext
    q̇0 = q̇next
    
    print('conf q : ', q0)
    print('velocity qdot: ', q̇0)
    
    