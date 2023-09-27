# PLOTS
import matplotlib.pyplot as plt
import numpy as np

def single_line_plot(x_timearr, y_valueplot, xlabel, ylabel, legend_names, title, file_name, file_path):
    # Plots simple XY plot
    #y_valueplot = np.array(y_valueplot)
    plt.figure(figsize=(10, 5))
    plt.plot(x_timearr, y_valueplot)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.title(title)
    plt.legend([legend_names], fontsize="x-large")

    # Save the plot as a PNG image
    plt.savefig(file_path + '/' + file_name + '.png', format='png')

    plt.show()

def multi_line_plot(x_timearr, y_valueplot, xlabel, ylabel, legend_names, title, file_name, file_path):
    # torque plot
    plt.figure(figsize=(10, 5))

    if isinstance(y_valueplot, np.ndarray):
        # If y_valueplot is a NumPy array
        plt.plot(x_timearr, y_valueplot[:, 0])
        plt.plot(x_timearr, y_valueplot[:, 1], color='red')
    else:
        # If y_valueplot is a list of lists
        plt.plot(x_timearr, [y[0] for y in y_valueplot])
        plt.plot(x_timearr, [y[1] for y in y_valueplot], color='red')

    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.title(title)
    plt.legend([legend_names[0], legend_names[1]], fontsize="x-large")

    # Save the plot as a PNG image
    plt.savefig(file_path + '/' + file_name + '.png', format='png')

    plt.show()



