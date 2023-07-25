import pinocchio as pin
import numpy as np

# Compute the potential and kinetic energy of the Acrobot system
def compute_energy(robotModel, ms, ls, lc, gravity, Is, q, qdot):

    g = gravity
    #I1 = m1 * l1 ** 2 / 12
    #I2 = m2 * l2 ** 2 / 12

    t1 = ms[0]*lc[0]*lc[0] + ms[1]*ls[0]*ls[0] + Is[0]
    t2 = ms[1]*lc[1]*lc[1] + Is[1]
    t3 = ms[1]*ls[0]*lc[1]
    t4 = ms[0]*lc[0] + ms[1]*ls[0]
    t5 = ms[1]*lc[1]

    ts = np.array([t1, t2, t3, t4,t5]) #note in this case beta are without g

    kv = 45.0
    ke = 0.5
    kd = 15.0
    kp = 22.0
    
    return acrobot_energy, desired_energy, ts[:3], ts[-2:], M, C, G, gains