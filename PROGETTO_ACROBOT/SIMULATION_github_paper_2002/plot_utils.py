import matplotlib.pyplot as plt
import csv


def init_plot(plt, xAxisLabel, yAxisLabel, title):
    # Add labels and title
    plt.set_xlabel(xAxisLabel)
    plt.set_ylabel(yAxisLabel)
    plt.set_title(title)
        

def update_plot(plt, x, y, xAxisLabel, yAxisLabel, title, fileName):
    # Clear the plot before to add a new point
    plt.clear()

    # Update the data
    #plt.plot( x, y, '-bo')
    plt.plot( x, y)

    # Set labels and title
    plt.set_xlabel(xAxisLabel)
    plt.set_ylabel(yAxisLabel)
    plt.set_title(title)

    # Save the plot as a PNG image
    plt.figure.savefig(fileName + '.png', format='png')

    # Display the plot
    #plt.pause(0.001)
    plt.figure.canvas.draw()
    plt.figure.canvas.flush_events()



def update_2line_plot(plt, x, y1, y2, xAxisLabel, yAxisLabel, label1, label2, title, fileName):
    # Clear the plot before to add a new point
    plt.clear()

    # Set labels and title
    plt.set_xlabel(xAxisLabel)
    plt.set_ylabel(yAxisLabel)
    plt.set_title(title)

    #plt.plot(x, y1, '-o', color='blue', label = label1)
    #plt.plot(x, y2, '-o', color='red', label = label2)

    plt.plot(x, y1, color='blue', label = label1)
    plt.plot(x, y2, color='red', label = label2)

    legend = plt.get_legend()
    if not legend:
        plt.legend()
        plt.legend(loc='center right')

    # Save the plot as a PNG image
    plt.figure.savefig(fileName + '.png', format='png')

    # Display the plot
    #plt.pause(0.001)
    plt.figure.canvas.draw()
    plt.figure.canvas.flush_events()


def csv_write(x, y, fileName):
    with open(fileName + '.csv', 'a', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter=';')
        if csvfile.tell() == 0:
            writer.writerow(['X', 'Y'])
        writer.writerow([x, y])
        
def csv_write_multiple(x, y1, y2, fileName):
    # Save the points as a CSV file
    with open(fileName + '.csv', 'a', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter=';')
        if csvfile.tell() == 0:
            writer.writerow(['X', 'Y1', 'Y2'])
        writer.writerow([x, y1, y2])

    

    



