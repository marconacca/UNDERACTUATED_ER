# Main
import simulation
import bulletSimulation

#for the graphical part
import numpy as np
import matplotlib.pyplot as plt
from animation_utils import *

num_step, q_history, qdot_history, state_history = simulation.simulate()

simDT = 1/240
simTime = 10
fig, ax = plt.subplots(figsize=(10, 10))

#Disattivare se non si vuole la simulazione con PyBullet
bulletSimulation.simulate(state_history,  simDT, simTime)

#Disattivare se non si vuole l'animazione 2D'
animate01([1.0,2.0], fig, ax, simTime, simDT, state_history, save=True, show_time=True)
#animate00([1.0,2.0], ax, di, simDT, num_step, q_history)


print('END')






    
