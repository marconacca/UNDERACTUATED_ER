# Main

import simulation

#for the graphical part
import numpy as np
import matplotlib.pyplot as plt




num_step, q_history, _ = simulation.simulate()




# -------------------- TO CREATE THE PNG IMAGES FOR THE GRAPHIC GIF SIMULATION --------------------
# Make an image every di time points, corresponding to a frame rate of fps
# frames per second.
# Frame rate, s-1

# L1 = 1.0
# L2 = 2.0

# # Convert to Cartesian coordinates of the two bob positions.
# x1 = L1 * np.sin(q_history[:,0])
# y1 = -L1 * np.cos(q_history[:,0])
# x2 = x1 + L2 * np.sin(q_history[:,1])
# y2 = y1 - L2 * np.cos(q_history[:,1])

# simDT = 1/240
# fps = 10
# #di = int(1/fps/(1/240))
# di = 20
# fig = plt.figure(figsize=(8.3333, 6.25), dpi=72)
# ax = fig.add_subplot(111)

# for i in range(0, num_step, di):
#      #print(i // di, '/', num_step // di)
#     simulation.make_plot(i, ax, x1, x2, y1, y2, L1, L2, simDT, di)
    
