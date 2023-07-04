import pinocchio as pin
import numpy as np

# Compute the potential and kinetic energy of the Acrobot system
def compute_energy(robotModel, q, qdot):
    # data = model.createData()

    # Set joint positions and velocities
    pin.forwardKinematics(robotModel.model, robotModel.data, q, qdot)

    # Compute joint Jacobians and joint Jacobian time variation
    pin.computeJointJacobians(robotModel.model, robotModel.data, q)
    pin.computeJointJacobiansTimeVariation(robotModel.model, robotModel.data, q, qdot)
    # updates the position of each frame contained in the mode
    pin.updateFramePlacements(robotModel.model, robotModel.data)

    # Compute the dynamics matrices
    M = pin.crba(robotModel.model, robotModel.data, q)
    C = pin.computeCoriolisMatrix(robotModel.model, robotModel.data, q, qdot)
    G = pin.computeGeneralizedGravity(robotModel.model, robotModel.data, q)

    # Compute the kinetic energy
    kinetic_energy = pin.computeKineticEnergy(robotModel.model, robotModel.data, q, qdot)
    #kinetic_energy = 0.5 * np.dot(qdot, np.dot(M, qdot))
    

    # Compute the potential energy
    #potential_energy = pin.computePotentialEnergy(robotModel.model, robotModel.data, q, qdot)
    gravity = robotModel.model.gravity.linear[2]  # Assuming gravity is in the z-direction
    potential_energy = 0.0
    for i in range(robotModel.model.njoints):
        com_pos = robotModel.data.oMi[i].translation
        mass = robotModel.model.inertias[i].mass
        potential_energy += mass * gravity * com_pos[2]

    acrobot_energy = kinetic_energy + potential_energy

    



    return acrobot_energy, M, C, G