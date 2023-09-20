import numpy as np

def stabilization_controller(q, qdot, state_error):
    # we should use a LQR control to stabilize at the end the acrobot (putting a threshold of the error in position)

    #state_error_paper = np.array([q[0]-np.pi/2, q[1], qdot[0], qdot[1]])


    # Matrice K of LQR controller of the PAPER parameters
    K = np.array([-246.481, -98.690, -106.464, -50.138])

    # matrice K for our model parameters
    #K = np.array([-460.5540, -171.0658,  -69.2076,  -26.9682])


    # Compute control torques
    tau2 = np.dot(-K, state_error)
    

    control_torques = np.array([0, tau2])



    return control_torques