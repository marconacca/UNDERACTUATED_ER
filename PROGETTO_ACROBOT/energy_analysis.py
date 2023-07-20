import pinocchio as pin
import numpy as np

# Compute the potential and kinetic energy of the Acrobot system
def compute_energy(robotModel, q, qdot):
    # data = model.createData()

    # # # Set joint positions and velocities
    # pin.forwardKinematics(robotModel.model, robotModel.data, q, qdot)

    # # Compute joint Jacobians and joint Jacobian time variation
    # pin.computeJointJacobians(robotModel.model, robotModel.data, q)
    # pin.computeJointJacobiansTimeVariation(robotModel.model, robotModel.data, q, qdot)
    # # updates the position of each frame contained in the mode
    # pin.updateFramePlacements(robotModel.model, robotModel.data)


# #########################   Arm Parameters   #########################

    # # define parameters for energy computation
    # l1 = 0.1425
    # l2 = 0.2305
    # lc1 = 0.035
    # lc2 = 0.1
    l1 = 0.102611
    l2 = 0.220227
    lc1 = 0.0370271
    lc2 = 0.101004
    m1 = 0.26703
    m2 = 0.33238
    # inertia for each links (other values are smaller than 10^-8, I keep until 10^-5)
    I1 = np.matrix([[0.00040827, 0, 0.000018738], [0, 0.00038791, 0], [0.000018738, 0, 0.000036421]])
    I2 = np.matrix([[0.0011753, 0, 0], [0, 0.0011666, 0], [0, 0, 0.000014553]])
    I1 = I1[0,0]  # x-axis inertia
    I2 = I2[0,0]
    #I1 = I1[2,2]  # z-axis inertia
    #I2 = I2[2,2]
    g = 9.81


    # Paper parameters
    # l1 = 1
    # l2 = 2
    # lc1 = 0.5
    # lc2 = 1
    # m1 = 1
    # m2 = 1
    # I1 = 0.083
    # I2 = 0.33
    # g = 9.8


# #########################   DYNAMICS PARAMETERS and MATRICES   #########################

    alpha1 = m1*(lc1**2) + m2*(l1**2) + I1
    alpha2 = m2*(lc2**2) + I2
    alpha3 = m2*l1*lc2
    beta1 = (m1*lc1 + m2*l1)*g
    beta2 = m2*lc2*g


    # @@@@@@@@@@   PINOCCHIO DYNAMICS   @@@@@@@@@@
    # M = pin.crba(robotModel.model, robotModel.data, q)
    # C = pin.computeCoriolisMatrix(robotModel.model, robotModel.data, q, qdot)
    # G = pin.computeGeneralizedGravity(robotModel.model, robotModel.data, q)

    #pin.computeAllTerms(robotModel.model, robotModel.data, q, qdot)
    # M = robotModel.data.M
    # C = robotModel.data.C
    # G = robotModel.data.g

    # print('q ', q)
    # print('qdot ', qdot)

    
    # @@@@@@@@@@   PAPERS DYNAMICS   @@@@@@@@@@
    M = np.array([[alpha1+alpha2+2*alpha3*np.cos(q[1]), alpha2+alpha3*np.cos(q[1])],
                    [alpha2+alpha3*np.cos(q[1]), alpha2]])
    C = alpha3*np.array([-2*qdot[0]*qdot[1] - qdot[1]**2, qdot[0]**2])*np.sin(q[1])
    G = np.array([beta1*np.cos(q[0]) + beta2*np.cos(q[0]+q[1]), beta2*np.cos(q[0]+q[1])])

    #M_det = np.linalg.det(M)
    M_det = alpha1*alpha2 - (alpha3**2)*((np.cos(q[1]))**2)


    # compute the DESIRED ENERGY (potential energy up-right position)
    desired_energy = beta1 + beta2


# #########################   Compute Gains threshold to choose the gains kp, kd, kv   #########################

    # Compute the threshold for kd gain
    def max_f_kd(q2):
        phi2 = np.sqrt(beta1**2+beta2**2+2*beta1*beta2*np.cos(q2))
        return (phi2 + desired_energy)*(alpha1*alpha2 - (alpha3**2)*(np.cos(q2)**2))/(alpha1+alpha2+2*alpha3*np.cos(q2)) 
    
    q2_values = np.linspace(0, 2*(np.pi), 1000)

    thresh_kd_vec = max_f_kd(q2_values)
    thresh_kd = max( thresh_kd_vec)
    print('-----thresh_kd: ', thresh_kd)

    # Compute the threshold for the kp gain
    thresh_kp = (2/np.pi)*min(beta1**2, beta2**2)
    print('-----thresh_kp: ', thresh_kp)


    #gains 1 condition
    #kd > max( ((phi2 + desired_energy)*M_det)/(M[0,0]) )    #for q2 belongs [0, 2pi]     (25 formula in paper)
    #kp > (2/np.pi)*min(beta1**2, beta2**2)          #(43 formula in paper)
    #kv > 0


    # Define controller GAINS with our integration of the states (with x-inerta instead of z-inertia)
    kp = 0.075    # Proportional gain    > 0.06905021
    kd = 0.004   # Dynamics gain       > 0.00365861
    kv = 0.1    # Derivative gain
    gains = np.array([kp, kd, kv])
    # Define controller GAINS with getJointState pybullet function
    # kp = 0.22    # Proportional gain    > 0.1970336
    # kd = 0.016   # Dynamics gain       > 0.0.0143357
    # kv = 2    # Derivative gain
    # gains = np.array([kp, kd, kv])

    # Define controller GAINS with our integration of the states  (with x-inertia and alex link length)
    #kp = 0.07    # Proportional gain    > 0.06905021
    #kd = 0.006   # Dynamics gain       > 0.00365861
    #kv = 0.08    # Derivative gain
    #gains = np.array([kp, kd, kv])
    

    # Paper Gains
    # kp = 0.61    # Proportional gain
    # kd = 0.358   # Dynamics gain
    # kv = 0.663   # Derivative gain
    # kp = 61.2    # Proportional gain
    # kd = 35.8    # Dynamics gain
    # kv = 66.3    # Derivative gain
    # gains = np.array([kp, kd, kv])


# #########################   COMPUTES ENERGIES (already done Desired_Energy)   #########################

    # Compute the kinetic energy
    #kinetic_energy = pin.computeKineticEnergy(robotModel.model, robotModel.data, q, qdot)
    #M = np.squeeze(np.asarray(M))
    kinetic_energy = 0.5 * np.dot(qdot, np.dot(qdot, M))
    

    # Compute the POTENTIAL ENERGY
    potential_energy = beta1*np.sin(q[0]) + beta2*np.sin(q[0]+q[1])

    # PINOCCHIO POTENTIAL ENERGY
    # gravity = robotModel.model.gravity.linear[2]  # Assuming gravity is in the z-direction
    # potential_energy = 0.0
    # for i in range(robotModel.model.njoints):
    #     com_pos = robotModel.data.oMi[i].translation
    #     mass = robotModel.model.inertias[i].mass
    #     potential_energy += mass * gravity * com_pos[2]

    # TOTAL ENERGY of the Acrobot system
    acrobot_energy = kinetic_energy + potential_energy
    


    return acrobot_energy, desired_energy, M, C, G, M_det, gains