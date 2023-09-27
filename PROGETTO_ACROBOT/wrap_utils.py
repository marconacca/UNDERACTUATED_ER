import numpy as np


def wrap_angles(x):
    y = np.copy(x)
    y[0] = x[0] % (2*np.pi)
    y[1] = x[1] % (2*np.pi)
    return y


def wrap_angles_top(x):
    y = np.copy(x)
    y[0] = x[0] % (2*np.pi)
    y[1] = (x[1] + np.pi) % (2*np.pi) - np.pi
    return y


def wrap_angles_diff(x):
    y = np.copy(x)
    y[0] = x[0] % (2*np.pi)
    y[1] = x[1] % (2*np.pi)
    while np.abs(y[0]) > np.pi:
        y[0] -= 2*np.pi
    while np.abs(y[1]) > np.pi:
        y[1] -= 2*np.pi
    return y


#normalize any angle value to the range [0, 2π)
def normalize_angles_2002(angles):
    normalized_angles = np.zeros_like(angles)
    
    for i in range(angles.shape[0]):
        angle = angles[i]
        # Normalize angles: θ′[i] = θ′tmp + (θ′tmp <= 0 ? 2*π : (θ′tmp > 2*π ? -2*π : 0.0))
        if angle <= 0:
            normalized_angles[i] = angle + 2 * np.pi
        elif angle > 2 * np.pi:
            normalized_angles[i] = angle - 2 * np.pi
        else:
            normalized_angles[i] = angle
    
    return normalized_angles

def normalize_min_pi(angles):
    π = np.pi 

    # List comprehension to create θs_disp list
    θs_disp = [θ - 2*π if θ > π else θ for θ in angles]
    wrapped_angles = np.array(θs_disp)
    return wrapped_angles
