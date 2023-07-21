import numpy as np

# Compute the potential and kinetic energy of the Acrobot system
def compute_energy22(robotModel, q, qdot):



# #########################   Arm Parameters   #########################


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


# #########################   DYNAMICS PARAMETERS and MATRICES   #########################

    alpha1 = m1*(lc1**2) + m2*(l1**2) + I1
    alpha2 = m2*(lc2**2) + I2
    alpha3 = m2*l1*lc2
    beta1 = (m1*lc1 + m2*l1)*g
    beta2 = m2*lc2*g


    
    # @@@@@@@@@@   PAPERS DYNAMICS   @@@@@@@@@@
    M = np.array([[alpha1+alpha2+2*alpha3*np.cos(q[1]), alpha2+alpha3*np.cos(q[1])],
                    [alpha2+alpha3*np.cos(q[1]), alpha2]])
    C = alpha3*np.array([-2*qdot[0]*qdot[1] - qdot[1]**2, qdot[0]**2])*np.sin(q[1])
    G = np.array([beta1*np.cos(q[0]) + beta2*np.cos(q[0]+q[1]), beta2*np.cos(q[0]+q[1])])

    #M_det = np.linalg.det(M)
    M_det = ( alpha1+alpha2+2*alpha3*np.cos(q[1]) ) * ( alpha2 ) - ( alpha2+alpha3*np.cos(q[1]) )




# #########################   Compute Gains threshold to choose the gains kp, kd, kv   #########################

  

    

    # Paper Gains
    kp = 22    # Proportional gain
    kd = 15    # Dynamics gain
    kv = 45   # Derivative gain
    ke = 0.5
    gains = np.array([kp, kd, kv, ke])


# #########################   COMPUTES ENERGIES (already done Desired_Energy)   #########################

    # Compute the kinetic energy
    #kinetic_energy = pin.computeKineticEnergy(robotModel.model, robotModel.data, q, qdot)
    #M = np.squeeze(np.asarray(M))
    kinetic_energy = 0.5 * np.dot(qdot.T, np.dot(qdot, M))
    

    # Compute the POTENTIAL ENERGY
    potential_energy = beta1*np.sin(q[0]) + beta2*np.sin(q[0]+q[1])

    # Compute the DESIRED ENERGY (potential energy up-right position)
    desired_energy = beta1 + beta2

    # TOTAL ENERGY of the Acrobot system
    acrobot_energy = kinetic_energy + potential_energy
    
    print("Kinetic energy :", kinetic_energy)
    print("Potential energy :", potential_energy)
    


    return acrobot_energy, desired_energy, M, C, G, M_det, gains