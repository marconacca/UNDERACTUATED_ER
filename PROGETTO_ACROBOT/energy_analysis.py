import pinocchio as pin
import numpy as np

# Compute the potential and kinetic energy of the Acrobot system
def compute_energy(robotModel, q, qdot):
    # data = model.createData()

    # Set joint positions and velocities
    pin.forwardKinematics(robotModel.model, robotModel.data, q, qdot)

    # Compute joint Jacobians and joint Jacobian time variation
    pin.computeJointJacobians(robotModel.model, robotModel.data)
    pin.computeJointJacobiansTimeVariation(robotModel.model, robotModel.data, q, qdot)
    # updates the position of each frame contained in the mode
    pin.updateFramePlacements(robotModel.model, robotModel.data)

    # Compute the kinetic energy
    M = pin.crba(robotModel.model, robotModel.data, q)
    kinetic_energy = 0.5 * np.dot(qdot, np.dot(M, qdot))

    # Compute the potential energy
    gravity = robotModel.model.gravity.linear[2]  # Assuming gravity is in the z-direction
    potential_energy = 0.0
    for i in range(robotModel.model.njoints - 1):
        com_pos = robotModel.data.oMi[i + 1].translation
        mass = robotModel.model.inertias[i + 1].mass
        potential_energy += mass * gravity * com_pos[2]

    return potential_energy, kinetic_energy