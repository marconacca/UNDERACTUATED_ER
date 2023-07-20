import numpy as np

def normalize_angle(angle):
    """
    Normalize an angle to the range [0, 2π].
    """
    return angle % (2 * np.pi)

def derivative(state, torques):
    """
    Compute the derivatives of the given state including torques.
    """
    angle1, angle2, angular_velocity1, angular_velocity2 = state
    torque1, torque2 = torques

    # Calculate angular accelerations based on torques and inertia parameters.
    I1 = 0.083
    I2 = 0.33
    angular_acceleration1 = (torque1 - torque2) / I1
    angular_acceleration2 = (torque2 - torque1) / I2

    return angular_velocity1, angular_velocity2, angular_acceleration1, angular_acceleration2

def euler_integrator(state, final_state, dt, torques):
        """
        Performs a Euler integration step

        Parameters
        ----------
        state : array_like, shape=(4,), dtype=float,
            state of the double pendulum,
            order=[angle1, angle2, velocity1, velocity2],
            units=[rad, rad, rad/s, rad/s]
        dt : float
            timestep, unit=[s]
        torques : array_like, shape=(2,), dtype=float
            actuation input/motor torque,
            order=[tau1, tau2],
            units=[Nm]

        Returns
        -------
        numpy_array
            shape=(4,), dtype=float,
            new state of the double pendulum,
            order=[angle1, angle2, velocity1, velocity2],
            units=[rad, rad, rad/s, rad/s]
        """


        # Unpack the initial state
        angle1, angle2, angular_velocity1, angular_velocity2 = state

        # Unpack the final state
        final_angle1, final_angle2, final_angular_velocity1, final_angular_velocity2 = final_state

        # Integrate the state using Euler's method
        # Update the state using Euler's method
        """
        angle1 += angular_velocity1 * dt
        angle2 += angular_velocity2 * dt
        angular_velocity1 += (final_angular_velocity1 - angular_velocity1) * dt
        angular_velocity2 += (final_angular_velocity2 - angular_velocity2) * dt

        return [normalize_angle(angle1), normalize_angle(angle2)], [angular_velocity1, angular_velocity2] 
        """

        derivatives = derivative(state, torques)
        updated_state = [state[i] + derivatives[i] * dt for i in range(4)]
        return tuple(normalize_angle(angle) for angle in updated_state[:2]), updated_state[2:]



def runge_integrator(state, final_state, dt, torques):
        """
        Performs a Runge-Kutta integration step

        Parameters
        ----------
        state : array_like, shape=(4,), dtype=float,
            state of the double pendulum,
            order=[angle1, angle2, velocity1, velocity2],
            units=[rad, rad, rad/s, rad/s]
        dt : float
            timestep, unit=[s]
        torques : array_like, shape=(2,), dtype=float
            actuation input/motor torque,
            order=[tau1, tau2],
            units=[Nm]

        Returns
        -------
        numpy_array
            shape=(4,), dtype=float,
            new state of the dacrobot,
            order=[angle1, angle2, velocity1, velocity2],
            units=[rad, rad, rad/s, rad/s]
        """

        """
        Perform a single step of the integration using the fourth-order Runge-Kutta method including torque.
        """
        k1 = [dt * deriv for deriv in derivative(state, torques)]
        k2 = [dt * deriv for deriv in derivative([state[i] + 0.5 * k1[i] for i in range(4)], torques)]
        k3 = [dt * deriv for deriv in derivative([state[i] + 0.5 * k2[i] for i in range(4)], torques)]
        k4 = [dt * deriv for deriv in derivative([state[i] + k3[i] for i in range(4)],torques)]
    
        updated_state = [state[i] + (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) / 6 for i in range(4)]

        return tuple(normalize_angle(angle) for angle in updated_state[:2]), updated_state[2:]
    