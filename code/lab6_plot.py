"""
Sample Script to generate a simple x-y plot
Use "python3 lab6_plot.py [path/to/csv/file.csv]" to execute
The csv-file should contain a triple per line:
    time,current_state,desired_state
"""

import numpy as np
import matplotlib
# if on the robot, don't use X backend
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_file", help="input file containing csv data")
    args = parser.parse_args()

    # load comma-separated table into a numpy matrix
    data = np.loadtxt(args.csv_file, delimiter=',')

    # plot using matplotlib
    fig = plt.figure()
    ax1 = fig.add_subplot(111)
    # optionally set limits for x/y axis
    # ax1.set_xlim([-1.1, 1.1])
    # ax1.set_ylim([-1.1, 1.1])
    # plot first column (t) vs. second column (current_state) in blue
    ax1.plot(data[:,0],data[:,1], "b", label="Current State")
    # plot first column vs. 3rd column in red
    ax1.plot(data[:,0],data[:,2], "r", label="Desired State")
    # save plot as file
    plt.legend()
    plt.xlabel("t [s]")
    plt.ylabel("angle [rad]")
    plt.savefig(args.csv_file + ".png")
