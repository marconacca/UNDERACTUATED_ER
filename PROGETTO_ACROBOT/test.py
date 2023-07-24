import numpy as np
import control as ctrl
import matplotlib.pyplot as plt

# Double pendulum parameters
m1 = 1.0  # Mass of the first pendulum (kg)
m2 = 1.0  # Mass of the second pendulum (kg)
l1 = 1.0  # Length of the first pendulum (m)
l2 = 1.0  # Length of the second pendulum (m)
g = 9.81  # Acceleration due to gravity (m/s^2)

# State-space representation of the double pendulum dynamics
A = np.array([[0, 1, 0, 0],
              [0, -(m1 + m2) * g / (m1 * l1 + m2 * l2), 0, 0],
              [0, 0, 0, 1],
              [0, m2 * g / (m1 * l1 + m2 * l2), 0, 0]])
B = np.array([[0],
              [(m1 + m2) / (m1 * l1 + m2 * l2)],
              [0],
              [-m2 / (m1 * l1 + m2 * l2)]])
C = np.eye(4)
D = np.zeros((4, 1))

# State-space model
sys = ctrl.StateSpace(A, B, C, D)

# Energy controller
Q = np.diag([10, 1, 10, 1])  # State weighting matrix
R = np.array([[1]])         # Control weighting matrix
K, _, _ = ctrl.lqr(sys, Q, R)

# Ensure K has the correct shape (1, 4)
K = K[0]

# Time vector
dt = 0.01
t = np.arange(0, 10, dt)

# Initial state (swing-up phase)
theta1 = np.pi - 0.1
theta2 = 0.0
theta1_dot = 0.0
theta2_dot = 0.0
x0 = np.array([theta1, theta1_dot, theta2, theta2_dot])

# Energy-based control law for swing-up phase
def energy_control(x):
    theta1, theta1_dot, theta2, theta2_dot = x
    V = m1 * l1 * g * (1 - np.cos(theta1)) + m2 * g * (1 - np.cos(theta2))
    V_dot = m1 * l1 * g * np.sin(theta1) * theta1_dot + m2 * g * np.sin(theta2) * theta2_dot
    u = np.array([V_dot - 0.1 * V])  # Energy-based control law with damping
    return u

# Simulation using Euler's method
# Simulation using Euler's method
states = [x0]
for i in range(len(t) - 1):
    u_swing_up = energy_control(states[-1])
    u_stabilize = -K @ states[-1].reshape(-1, 1)  # Reshape to a column vector
    u = u_swing_up + u_stabilize
    x_next = states[-1] + dt * sys.A @ states[-1] + dt * sys.B @ u
    states.append(x_next)

# Plot the results
theta1_values = [state[0] for state in states]
theta2_values = [state[2] for state in states]

plt.figure()
plt.plot(t, theta1_values, label="Theta1")
plt.plot(t, theta2_values, label="Theta2")
plt.xlabel("Time (s)")
plt.ylabel("Angles (rad)")
plt.legend()
plt.grid(True)
plt.title("Double Pendulum Swing-Up and Stabilization")
plt.show()