# PLOTS
import matplotlib.pyplot as plt
import numpy as np

# @@@@@@@@@@@@@@@   PLOTS TESTING   @@@@@@@@@@@@@@@

def plotting_conf(x_timearr, y_valueplot, xlabel, ylabel, legend_names, title):
    # q plot
    plt.figure(figsize=(10, 5))

    if isinstance(y_valueplot, np.ndarray):
        # If y_valueplot is a NumPy array
        plt.plot(x_timearr, y_valueplot[:, 0] - np.pi/2)
        plt.plot(x_timearr, y_valueplot[:, 1])
    else:
        # If y_valueplot is a list of lists
        plt.plot(x_timearr, [y[0] - np.pi/2 for y in y_valueplot])
        plt.plot(x_timearr, [y[1] for y in y_valueplot])

    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.title(title)
    plt.legend([legend_names[0], legend_names[1]], fontsize="x-large")
    plt.show()

    
def plotting_vel(x_timearr, y_valueplot, xlabel, ylabel, legend_names, title):
    # qdot plot
    plt.figure(figsize=(10, 5))

    if isinstance(y_valueplot, np.ndarray):
        # If y_valueplot is a NumPy array
        plt.plot(x_timearr, y_valueplot[:, 0])
        plt.plot(x_timearr, y_valueplot[:, 1])
    else:
        # If y_valueplot is a list of lists
        plt.plot(x_timearr, [y[0] for y in y_valueplot])
        plt.plot(x_timearr, [y[1] for y in y_valueplot])

    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.title(title)
    plt.legend([legend_names[0], legend_names[1]], fontsize="x-large")
    plt.show()

def plotting_torque(x_timearr, y_valueplot, xlabel, ylabel, legend_names, title):
    # torque plot
    plt.figure(figsize=(10, 5))

    if isinstance(y_valueplot, np.ndarray):
        # If y_valueplot is a NumPy array
        plt.plot(x_timearr, y_valueplot[:, 0] - np.pi/2)
        plt.plot(x_timearr, y_valueplot[:, 1])
    else:
        # If y_valueplot is a list of lists
        plt.plot(x_timearr, [y[0] - np.pi/2 for y in y_valueplot])
        plt.plot(x_timearr, [y[1] for y in y_valueplot])

    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.title(title)
    plt.legend([legend_names[0], legend_names[1]], fontsize="x-large")
    plt.show()



def plotting_singleval(x_timearr, y_valueplot, xlabel, ylabel, legend_names, title):
    # Plots simple XY plot
    #y_valueplot = np.array(y_valueplot)
    plt.figure(figsize=(10, 5))
    plt.plot(x_timearr, y_valueplot)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.title(title)
    plt.legend([legend_names], fontsize="x-large")
    plt.show()

    #plt.savefig('plot??.pdf')