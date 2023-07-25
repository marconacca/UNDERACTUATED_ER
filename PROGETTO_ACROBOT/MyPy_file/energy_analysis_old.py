import pinocchio as pin
import numpy as np

# Compute the potential and kinetic energy of the Acrobot system
def compute_energy(robotModel, mass, length, com, gravity, inertia, q, qdot):
    # data = model.createData()

    # # # Set joint positions and velocities
    # pin.forwardKinematics(robotModel.model, robotModel.data, q, qdot)

    # # Compute joint Jacobians and joint Jacobian time variation
    # pin.computeJointJacobians(robotModel.model, robotModel.data, q)
    # pin.computeJointJacobiansTimeVariation(robotModel.model, robotModel.data, q, qdot)
    # # updates the position of each frame contained in the mode
    # pin.updateFramePlacements(robotModel.model, robotModel.data)



# #########################   Dynamics Parameters   #########################

    l1 = length[0]
    l2 = length[1]
    lc1 = com[0]
    lc2 = com[1]
    m1 = mass[0]
    m2 = mass[1]
    g = gravity
    I1 = inertia[0]
    I2 = inertia[1]
    #I1 = m1 * l1 ** 2 / 3
    #I2 = m2 * l2 ** 2 / 3


    alpha1 = m1*(lc1**2) + m2*(l1**2) + I1
    alpha2 = m2*(lc2**2) + I2
    alpha3 = m2*l1*lc2

    alphas = np.array([alpha1, alpha2, alpha3])

    beta1 = (m1*lc1 + m2*l1)*g
    beta2 = m2*lc2*g

    betas = np.array([beta1, beta2])

    """
    if (alpha2*beta1) > (alpha3*beta2):
        print(f"alpha2*beta1 > alpha3*beta2 fulfills the property")
    else:
        print(f"alpha2*beta1 > alpha3*beta2 does NOT fulfill property")

    if (alpha3*beta1) >= (alpha1*beta2):
        print(f"alpha3*beta1 >= alpha1*beta2 fulfills the property")
    else:
        print(f"alpha3*beta1 >= alpha1*beta2 does NOT fulfill property")
    """
    


    #q: array_like, shape=(2,), dtype=float,
    #angles state of the double pendulum,
    #order=[angle1, angle2],
    #units=[rad, rad,]

    #qdot: array_like, shape=(2,), dtype=float,
    #velocity state of the double pendulum,
    #order=[velocity1, velocity2],
    #units=[rad/s, rad/s]

    q1 = q[0]
    q2 = q[1]
    dq1 = qdot[0]
    dq2 = qdot[1]

    M = np.array([[alpha1+alpha2+2*alpha3*np.cos(q2), alpha2+alpha3*np.cos(q2)],
                    [alpha2+alpha3*np.cos(q2), alpha2]]) #mass_matrix
    C = alpha3*np.array([-2*dq1*dq2 - dq2**2, dq1*2])*np.sin(q2) #coriolis_matrix
    G = np.array([beta1*np.cos(q1) + beta2*np.cos(q1+q2), beta2*np.cos(q1+q2)]) #gravity_vector

    #M_det = np.linalg.det(M)
    #delta = alpha1*alpha2 - (alpha3**2)*((np.cos(q2))**2)
    delta = (alpha1+alpha2+2*alpha3*np.cos(q2) ) * ( alpha2 ) - ( alpha2+alpha3*np.cos(q2) )


    #Er is the energy of Acrobot at the upright equilibrium point
    desired_energy = beta1 + beta2


# #########################   Compute Gains threshold to choose the gains kp, kd, kv   #########################

    """Set controller gains.

        Parameters
        ----------
        kp : float
            gain for position error
        kd : float
            gain
        kv : float
            gain for velocity error

    """
    # Paper Gains 2007
    #kp = 61.2    # Proportional gain
    #kd = 35.8    # Dynamics gain
    #kv = 1    # Derivative gain
    #ke = 1 #non usato

    # Paper Gains 2001
    kp = 22    # Proportional gain
    kd = 15   # Dynamics gain
    kv = 45   # Derivative gain
    ke = 0.5
    
    gains = np.array([kp, kd, kv, ke])
    

    #q2_values = np.linspace(0, 2*(np.pi), 1000) #for q2 belongs [0, 2pi] 
    phi_q2 = np.sqrt(beta1**2+beta2**2+2*beta1*beta2*np.cos(q2))

    kp_threshold = round((2/np.pi)*min(beta1**2, beta2**2))#(43) in paper
    
    #kd_threshold = max(((phi_q2 + desired_energy)*(alpha1*alpha2 - (alpha3**2)*(np.cos(q2)**2))/(alpha1+alpha2+2*alpha3*np.cos(q2_values))))#f(25) in paper
    kd_threshold = ((phi_q2 + desired_energy)*delta/(alpha1+alpha2+2*alpha3*np.cos(q2)))#f(25) in paper

    if kp > kp_threshold:
        print(f"Kp={kp} fulfills the convergence property kp > {kp_threshold}")
    else:
        print(f"Kp={kp} does NOT fulfill convergence property kp > {kp_threshold}")


    if kd > kd_threshold:
        print(f"Kd={kd} fulfills the convergence property kd > {kd_threshold}")
    else:
        print(f"Kd={kd} does NOT fulfill the convergence property kd > {kd_threshold}")

    if kv > 0:
        print(f"Kv={kv} fulfills the convergence property kv > 0")
    else:
        print(f"Kv={kv} does NOT fulfill the convergence property kv > 0")


# #########################   Compute Energies (already done Desired_Energy)   #########################

    # Compute the kinetic energy
    #kinetic_energy = pin.computeKineticEnergy(robotModel.model, robotModel.data, q, qdot)
    #M = np.squeeze(np.asarray(M))
    kinetic_energy =  0.5 * np.dot(qdot.T, np.dot(qdot, M))
    

    # Compute the potential energy
    potential_energy = beta1*np.sin(q[0]) + beta2*np.sin(q[0]+q[1])
    # gravity = robotModel.model.gravity.linear[2]  # Assuming gravity is in the z-direction
    # potential_energy = 0.0
    # for i in range(robotModel.model.njoints):
    #     com_pos = robotModel.data.oMi[i].translation
    #     mass = robotModel.model.inertias[i].mass
    #     potential_energy += mass * gravity * com_pos[2]

    # Total Energy of the Acrobot system
    acrobot_energy = kinetic_energy + potential_energy


    

    """
    # Calculate potential energy for each pendulum
    V1 = m1 * g * l1 * (1 - np.cos(q1))
    V2 = m2 * g * l2 * (1 - np.cos(q2))

    # Calculate the moment of inertia for each pendulum
    #I1 = m1 * l1 ** 2 / 3
    #I2 = m2 * l2 ** 2 / 3
    
    #Calculate the kinetic energy for each pendulum using the moment of inertia
    T1 = 0.5 * I1 * dq1 ** 2
    T2 = 0.5 * I2 * (dq1 ** 2 + dq2 ** 2 + 2 * l1 * l2 * dq1 * dq2 * np.cos(q1 - q2))

    

    # Calculate kinetic energy for each pendulum
    T1 = 0.5 * m1 * (l1 * dq1) ** 2
    T2 = 0.5 * m2 * ((l1 * dq1) ** 2 + (l2 * dq2) ** 2 + 2 * l1 * l2 * dq1 * dq2 * np.cos(q1 - q2))

    # Calculate the total potential energy V
    V = V1 + V2

    # Calculate V_dot (the rate of change of potential energy)
    V_dot = m1 * g * l1 * np.sin(q1) * dq1 + m2 * g * l2 * np.sin(q2) * dq2

    # Calculate the total energy E
    E = V1 + V2 + T1 + T2

    #acrobot_energy = E
    """
    


    return acrobot_energy, desired_energy, alphas, betas, M, C, G, gains