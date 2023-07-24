import numpy as np
import pinocchio as pin
from energy_shaping import energy_shaping_controller
from stabilization import stabilization_controller
from energy_analysis import compute_energy

# Implement the swing-up control algorithm using Pinocchio
def swing_up_control(model, q, qdot, model_pars):

    """EnergyController
    Energy-based controller for acrobot swingup 

    Parameters
    ----------
    mass : array_like, optional
        shape=(2,), dtype=float, default=[1.0, 1.0]
        masses of the double pendulum,
        [m1, m2], units=[kg]
    length : array_like, optional
        shape=(2,), dtype=float, default=[0.5, 0.5]
        link lengths of the double pendulum,
        [l1, l2], units=[m]
    com : array_like, optional
        shape=(2,), dtype=float, default=[0.5, 0.5]
        center of mass lengths of the double pendulum links
        [r1, r2], units=[m]
    damping : array_like, optional
        shape=(2,), dtype=float, default=[0.5, 0.5]
        damping coefficients of the double pendulum actuators
        [b1, b2], units=[kg*m/s]
    gravity : float, optional
        default=9.81
        gravity acceleration (pointing downwards),
        units=[m/s²]
    inertia : array_like, optional
        shape=(2,), dtype=float, default=[None, None]
        inertia of the double pendulum links
        [I1, I2], units=[kg*m²]
        if entry is None defaults to point mass m*l² inertia for the entry
    torque_limit : array_like, optional
        shape=(2,), dtype=float, default=[np.inf, np.inf]
        torque limit of the motors
        [tl1, tl2], units=[Nm, Nm]
    model_pars : 
        Can be used to set all model parameters above
        If provided, the model_pars parameters overwrite
        the other provided parameters
        (Default value=None)
    """

    if(not model_pars):
        # Paper parameters
        mass=[1.0, 1.0]
        length=[1, 2]
        com=[0.5, 1]
        damping=[0.1, 0.2]
        gravity=9.81
        inertia=[0.083, 0.33]
        torque_limit=[np.inf, np.inf]
    else:
        # Model parameters
        mass=[0.26703, 0.33238]
        length=[0.1425, 0.2305]
        com=[0.035, (0.1 + 0.1425)]
        damping=[0.1, 0.1]
        gravity=9.81
        # inertia for each links (other values are smaller than 10^-8, I keep until 10^-5)
        #I1 = np.matrix([[0.00040827, 0, 0.000018738], [0, 0.00038791, 0], [0.000018738, 0, 0.000036421]])
        #I2 = np.matrix([[0.0011753, 0, 0], [0, 0.0011666, 0], [0, 0, 0.000014553]])
        inertia=[None, None]
        torque_limit=[np.inf, np.inf]
          

    

    # Energy shaping control
    energy, desired_energy, V_dot, alphas, betas, M, C, G, gains = compute_energy(model, mass, length, com, gravity, inertia, q, qdot)
    control_torques_energy_shaping, energy_error = energy_shaping_controller(model, energy, desired_energy, V_dot, alphas, betas, q, qdot, M, C, G, gains)

    #energy = current total energy of the system
    #desired energy = total energy when the acrobot is at rest in vertical position (ideally only potential energy)

    return control_torques_energy_shaping, energy_error

def stabilization_control(model, q, qdot, desired_position, desired_velocity):

    # Stabilization control (LQR)
    control_torques_stabilization, state_error = stabilization_controller(q, qdot, desired_position, desired_velocity)

    return control_torques_stabilization, state_error

def switch(q, qdot):

    zeta = 0.04 #threshold to switch controller (Energy to LQR)
    x_state = np.array([q[0]-np.pi/2, q[1], qdot[0], qdot[1]])
    condition = (abs(x_state[0]) + abs(x_state[1]) + 0.1*(abs(x_state[2])) + 0.1*(abs(x_state[3])))
    if (condition < zeta):
        return True
    
    return False