import matplotlib.pyplot as plt
import csv


def init_plot(plt, xAxisLabel, yAxisLabel, title):
    # Add labels and title
    plt.set_xlabel(xAxisLabel)
    plt.set_ylabel(yAxisLabel)
    plt.set_title(title)
        




def update_plot(plt, x, y, xAxisLabel, yAxisLabel, title, fileName):
    # Clear the plot before to add a new point
    #plt.clear()

    # Update the data
    plt.plot(x, y, 'o', color='red')

    # Set labels and title
    plt.set_xlabel(xAxisLabel)
    plt.set_ylabel(yAxisLabel)
    plt.set_title(title)

    # Save the plot as a PNG image
    plt.figure.savefig(fileName + '.png', format='png')

    # Save the points as a CSV file
    with open(fileName + '.csv', 'a', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter=';')
        if csvfile.tell() == 0:
            writer.writerow(['X', 'Y'])
        writer.writerow([x, y])

    # Display the plot
    #plt.pause(0.001)
    plt.figure.canvas.draw()
    plt.figure.canvas.flush_events()



def update_2line_plot(plt, x, y1, y2, xAxisLabel, yAxisLabel, label1, label2, title, fileName):

    # Set labels and title
    plt.set_xlabel(xAxisLabel)
    plt.set_ylabel(yAxisLabel)
    plt.set_title(title)

    plt.plot(x, y1, 'o', color='blue', label = label1)
    plt.plot(x, y2, 'o', color='red', label = label2)

    legend = plt.get_legend()
    if not legend:
        plt.legend()
        plt.legend(loc='center right')

    # Save the plot as a PNG image
    plt.figure.savefig(fileName + '.png', format='png')

    # Save the points as a CSV file
    with open(fileName + '.csv', 'a', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter=';')
        if csvfile.tell() == 0:
            writer.writerow(['X', 'Y1', 'Y2'])
        writer.writerow([x, y1, y2])

    plt.figure.canvas.draw()
    plt.figure.canvas.flush_events()

    



