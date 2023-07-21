# PLOTS
import matplotlib.pyplot as plt
import numpy as np

# @@@@@@@@@@@@@@@   PLOTS TESTING   @@@@@@@@@@@@@@@

def plotting_conf(x_timearr, y_valueplot, xlabel, ylabel, legend_names, title):
    # 1st joint configuration
    plt.figure(figsize=(10, 5))
    plt.plot(x_timearr, y_valueplot[:,0] - np.pi/2)
    plt.plot(x_timearr, y_valueplot[:,1])
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.title(title)
    plt.legend([legend_names[0], legend_names[1]], fontsize="x-large")
    plt.show()
    
def plotting_vel(x_timearr, y_valueplot, xlabel, ylabel, legend_names, title):
    # 1st joint configuration
    plt.figure(figsize=(10, 5))
    plt.plot(x_timearr, y_valueplot[:,0])
    plt.plot(x_timearr, y_valueplot[:,1])
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.title(title)
    plt.legend([legend_names[0], legend_names[1]], fontsize="x-large")
    plt.show()

def plotting_torque(x_timearr, y_valueplot, xlabel, ylabel, legend_names, title):
    # 1st joint configuration
    plt.figure(figsize=(10, 5))
    plt.plot(x_timearr, y_valueplot[:,0])
    plt.plot(x_timearr, y_valueplot[:,1])
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.title(title)
    plt.legend([legend_names[0], legend_names[1]], fontsize="x-large")
    plt.show()



def plotting_singleval(x_timearr, y_valueplot, xlabel, ylabel, legend_names, title):
    # 1st joint configuration
    plt.figure(figsize=(10, 5))
    plt.plot(x_timearr, y_valueplot)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.title(title)
    plt.legend([legend_names], fontsize="x-large")
    plt.show()

    #plt.savefig('plot??.pdf')