# Main

import simulation

#for the graphical part
import numpy as np
import matplotlib.pyplot as plt
from animation_utils import *

num_step, q_history, qdot_history, state_history = simulation.simulate()


simDT = 1/240
di = 10
fig, ax = plt.subplots(figsize=(10, 10))


#animate00([1.0,2.0], ax, di, simDT, num_step, q_history)


animate01([1.0,2.0], fig, ax, di, simDT, state_history, save=True, show_time=True)


print('END')






    
