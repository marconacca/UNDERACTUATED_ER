import shutil
import numpy as np
import math
import pinocchio as pin
import csv
import os

counter = 0


def csv_write(x, y, fileName):
    with open(fileName + '.csv', 'a', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter=';')
        if csvfile.tell() == 0:
            writer.writerow(['X', 'Y'])
        writer.writerow([x, y])
        
def csv_write_multiple(x, y1, y2, y3, y4, fileName):
    # Save the points as a CSV file
    with open(fileName + '.csv', 'a', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter=';')
        if csvfile.tell() == 0:
            writer.writerow(['X', 'q1', 'q2', 'q1dot', 'q1dot'])
        writer.writerow([x, y1, y2, y3, y4])


def energy_shaping_controller(current_energy, desired_energy, q, qdot, M, C, G, M_det, gains):
    global counter
    '''
    The current energy is the current (at a time instant) total energy of the system
    The desired energy is the total energy when the acrobot is at rest in vertical position (so just potential energy in theory)

    '''
    
    # ***************  Defining Parameters for the Control Law   ***************

    # dynamics and control components
    #M_det = np.linalg.det(M)
    #coriolis_forces = np.dot(C, qdot)
    h1 = C[0]
    h2 = C[1]


    kp = gains[0]
    kd = gains[1]
    kv = gains[2]
    # if using paper 2002
    # ke = gains[3]

    # Compute the error between desired energy and current energy
    energy_error = current_energy - desired_energy


    # #########################   CONTROL LAW tau2   #########################

    # Torque OLD PAPER 2002
    # nom = - ((kv*qdot[1] + kp*q[1])*M_det + kd*(M[1,0]*(h1+G[0])-M[0,0]*(h2+G[1])))
    # den =  ke*energy_error*M_det + kd*M[0,0]
    # tau2 = nom / den
    
    # Torque PAPER 2007
    nom = - ((kv*qdot[1] + kp*q[1])*M_det + kd*(M[1,0]*(h1+G[0])-M[0,0]*(h2+G[1])))
    den = kd*M[0,0] + energy_error*M_det
    tau2 = nom / den
    
    
    
    # # ----- Torque limitation -----
    # if (abs(tau2) > 25.0):
    #     sign = np.sign(tau2)
    #     tau2 = 25*sign
    
    control_torques = np.array([0., tau2])


    # ________________________________________________________________________________________
    # #########################   Values Printing Check and Saving   #########################

    #print('\nq configuration is: ', q)
    #print('qdot joint velocity is: ', qdot)
    print('\nM is: ', M)
    print('C is: ', C)
    print('G is: ', G)
    print('M_det is: ', M_det)
    print("\nCurrent energy: ", current_energy)
    print("Desired energy: ", desired_energy)
    print('€€€€€ - Energy Error is: ', energy_error)
    #print('  Tau2 is: ', tau2)
        # Create a directory for the files
    folder_path ='csv'

# Generate a unique filename for each plot
    filename0 = os.path.join(folder_path, f'state')
    filename1 = os.path.join(folder_path, f'inertia')
    filename2 = os.path.join(folder_path, f'Coriolis')
    filename3 = os.path.join(folder_path, f'Gravity')
    filename4 = os.path.join(folder_path, f'inertia_det')
    filename5 = os.path.join(folder_path, f'current_energy')
    filename6 = os.path.join(folder_path, f'energy_error')
    filename7 = os.path.join(folder_path, f'control_torques')
    filename8 = os.path.join(folder_path, f'q')
    filename9 = os.path.join(folder_path, f'qdot')
    
    csv_write(counter, np.concatenate((q,qdot)), filename0)
    csv_write(counter, np.squeeze(np.asarray(M)), filename1)
    csv_write(counter, C, filename2)
    csv_write(counter, G, filename3)
    csv_write(counter, M_det, filename4)
    csv_write(counter, current_energy, filename5)
    csv_write(counter, energy_error, filename6)
    csv_write(counter, control_torques,filename7)
    csv_write(counter, q, filename8)
    csv_write(counter, qdot, filename9)
    
    counter += 1
    # ________________________________________________________________________________________
    
    
    return control_torques, energy_error