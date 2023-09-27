import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib.transforms import Affine2D
import imageio
from wrap_utils import *



def animate01(robotParam, fig, ax, T, dt, state, save=False, show_time=True):
        """
        Simulates the system until time T and animates it in
        a matplotlib figure
        """

        l1 = robotParam[0]
        l2 = robotParam[1]
        x = np.array(state)

        time_steps = np.arange(0, T, dt)
        n = time_steps.shape[0]
        states_cache = np.zeros((n, 4))
        for i, t in enumerate(time_steps):
            states_cache[i, :] = x[i, :]

        plt.ion()
        plt.axis("off")
        ax.axis("equal")
        ax.set_xlim(-1.1 * (l1 + l2), 1.1 * (l1 + l2))
        ax.set_ylim(-1.1 * (l1 + l2), 1.1 * (l1 + l2))

        ax.scatter([0], [0], marker="o", c="k")
        ax.plot([-l1, l1], [0, 0], c="k", linestyle="-")
        #Attivare se si vuole rotare l'asse in verticale, io lo preferivo in orizzonale
        #line = ax.plot([-l1 / 2, l1 / 2], [0, 0], c="k", linestyle="--", )
        #trans = Affine2D().rotate_deg(90)
        #line[0].set_transform(trans + ax.transData)
        p1, = ax.plot([], [], c="k", lw=2)
        p2, = ax.plot([], [], c="k", lw=2)
        middle_joint_p = ax.scatter([], [], c="b", zorder=10)
        final_joint_p = ax.scatter([], [], c="r", zorder=10)
        

        if show_time:
            time_text = ax.text(x=0, y=1.2*(l1 + l2), s="t=0")
        plt.show()

        counter=0
        images = []
        os.makedirs('frames/animate01', exist_ok=True)
        os.makedirs('frames/animate01/gif', exist_ok=True)

        for i, t in enumerate(time_steps):
            if i % 10 != 0:
                continue
            #Disattivare se si vuole l'orientamento originale, io lo preferivo verso il basso
            states_cache[i, :0] = states_cache[i, :0] + np.pi/2
            states_cache[i, 0:1] = states_cache[i, 0:1] + np.pi/2

            p1.set_data([0, l1 * np.sin(states_cache[i, 0])], [0, -l1 * np.cos(states_cache[i, 0])])
            p2.set_data([l1 * np.sin(states_cache[i, 0]),
                         l1 * np.sin(states_cache[i, 0]) + l2 * np.sin(states_cache[i, 0] + states_cache[i, 1])],
                        [-l1 * np.cos(states_cache[i, 0]),
                         -l1 * np.cos(states_cache[i, 0]) - l2 * np.cos(states_cache[i, 0] + states_cache[i, 1])])
            middle_joint_p.set_offsets([[l1 * np.sin(states_cache[i, 0]), -l1 * np.cos(states_cache[i, 0])]])
            final_joint_p.set_offsets([[l1 * np.sin(states_cache[i, 0]) + l2 * np.sin(states_cache[i, 0] + states_cache[i, 1]), -l1 * np.cos(states_cache[i, 0]) - l2 * np.cos(states_cache[i, 0] + states_cache[i, 1])]])

            if show_time:
                time_text.set_text(f"t={np.round(t, 1)}")
            fig.canvas.draw()

            plt.pause(dt)

            plt.savefig("frames/animate01/" + str(counter)+ ".png")
            images.append(imageio.imread("frames/animate01/" + str(counter)+ ".png"))
            counter += 1

        imageio.mimsave("frames/animate01/gif/double_pendulum.gif", images)



# -------------------- TO CREATE THE PNG IMAGES FOR THE GRAPHIC GIF SIMULATION --------------------
# Make an image every di time points, corresponding to a frame rate of fps
# frames per second.
# Frame rate, s-1

            
def animate00(robotParam, ax, di, simDT, num_step, q_history):
    # The pendulum rods.

    L1 = robotParam[0]
    L2 = robotParam[1]

    q_history = np.array(q_history)
    x1 = L1 * np.sin(q_history[:,0])
    y1 = -L1 * np.cos(q_history[:,0])
    x2 = x1 + L2 * np.sin(q_history[:,1])
    y2 = y1 - L2 * np.cos(q_history[:,1])

    os.makedirs('frames/animate00', exist_ok=True)
    os.makedirs('frames/animate00/gif', exist_ok=True)

    for i in range(0, num_step, di):
        # Plot and save an image of the double pendulum configuration for time
        # point i.

        # fps = 10
        #di = int(1/fps/(1/240))

    

        # Plotted bob circle radius
        r = 0.05
        # Plot a trail of the m2 bob's position for the last trail_secs seconds.
        trail_secs = 1
        # This corresponds to max_trail time points.
        max_trail = int(trail_secs / simDT)
    

        ax.plot([0, x1[i], x2[i]], [0, y1[i], y2[i]], lw=2, c='k')
        # Circles representing the anchor point of rod 1, and bobs 1 and 2.
        c0 = Circle((0, 0), r/2, fc='k', zorder=10)
        c1 = Circle((x1[i], y1[i]), r, fc='b', ec='b', zorder=10)
        c2 = Circle((x2[i], y2[i]), r, fc='r', ec='r', zorder=10)

        ax.add_patch(c0)
        ax.add_patch(c1)
        ax.add_patch(c2)

        # The trail will be divided into ns segments and plotted as a fading line.
        ns = 20
        s = max_trail // ns

        images = []

        for j in range(ns):
            imin = i - (ns-j)*s
            if imin < 0:
                continue
            imax = imin + s + 1
            # The fading looks better if we square the fractional length along the
            # trail.
            alpha = (j/ns)**2
            ax.plot(x2[imin:imax], y2[imin:imax], c='r', solid_capstyle='butt',
                lw=2, alpha=alpha)
            
        # Centre the image on the fixed anchor point, and ensure the axes are equal
        ax.set_xlim(-L1-L2-r, L1+L2+r)
        ax.set_ylim(-L1-L2-r, L1+L2+r)
        ax.set_aspect('equal', adjustable='box')
        plt.axis('off')
        plt.savefig('frames/animate00/_img{:04d}.png'.format(i//di), dpi=72)
        images.append(imageio.imread('frames/animate00/_img{:04d}.png'.format(i//di)))
        plt.cla()
    
    imageio.mimsave("frames/animate00/gif/double_pendulum.gif", images)
    

    

    
    
            