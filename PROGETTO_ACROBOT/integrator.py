def euler_integrator(self, state, dt, t, tau):
        """
        Performs a Euler integration step

        Parameters
        ----------
        y : array_like, shape=(4,), dtype=float,
            state of the double pendulum,
            order=[angle1, angle2, velocity1, velocity2],
            units=[rad, rad, rad/s, rad/s]
        dt : float
            timestep, unit=[s]
        t : float
            time, unit=[s]
        tau : array_like, shape=(2,), dtype=float
            actuation input/motor torque,
            order=[u1, u2],
            units=[Nm]

        Returns
        -------
        numpy_array
            shape=(4,), dtype=float,
            new state of the double pendulum,
            order=[angle1, angle2, velocity1, velocity2],
            units=[rad, rad, rad/s, rad/s]
        """
        return 1 #self.plant.rhs(t, y, tau)

def runge_integrator(self, y, dt, t, tau):
        """
        Performs a Runge-Kutta integration step

        Parameters
        ----------
        y : array_like, shape=(4,), dtype=float,
            state of the double pendulum,
            order=[angle1, angle2, velocity1, velocity2],
            units=[rad, rad, rad/s, rad/s]
        dt : float
            timestep, unit=[s]
        t : float
            time, unit=[s]
        tau : array_like, shape=(2,), dtype=float
            actuation input/motor torque,
            order=[u1, u2],
            units=[Nm]

        Returns
        -------
        numpy_array
            shape=(4,), dtype=float,
            new state of the double pendulum,
            order=[angle1, angle2, velocity1, velocity2],
            units=[rad, rad, rad/s, rad/s]
        
        k1 = self.plant.rhs(t, y, tau)
        k2 = self.plant.rhs(t + 0.5 * dt, y + 0.5 * dt * k1, tau)
        k3 = self.plant.rhs(t + 0.5 * dt, y + 0.5 * dt * k2, tau)
        k4 = self.plant.rhs(t + dt, y + dt * k3, tau)
        return (k1 + 2.0 * (k2 + k3) + k4) / 6.0
        """
        return 1