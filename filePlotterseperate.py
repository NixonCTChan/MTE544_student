# You can use this file to plot the loged sensor data
# Note that you need to modify/adapt it to your own files
# Feel free to make any modifications/additions here

import matplotlib.pyplot as plt
import numpy as np
from utilities import FileReader
import csv
import os

def format_title(filename):
    base_name = os.path.basename(filename)
    parts = base_name.split('_')
    if len(parts) >= 2:
        return f"{parts[0].title()} Data {parts[-1].split('.')[0].title()}"
    return base_name.title()

def plot_errors(filename):
    headers, values = FileReader(filename).read_file()
    
    time_list = []
    first_stamp = values[0][-1]
    
    if "laser" in filename:
        content_headers, content_values = FileReader(filename.replace("laser_ranges_content", "laser_content")).read_file()
        ranges_headers, ranges_values = FileReader(filename.replace("laser_content", "laser_ranges_content")).read_file()
        time_list = []
        cartesian_lasers = []
        first_stamp = content_values[0][-1]
        angle = content_values[0][0]
        angles = np.linspace(0, angle * len(ranges_values[0][0]), len(ranges_values[0][0]))
        angles = np.array([angles])
        for val in zip(content_values, ranges_values):
            time_list.append(val[0][1] - first_stamp)
            ranges = np.array(val[1])
            mask = ranges != np.inf
            ranges = ranges[mask]
            cartesian_lasers.append([ranges*np.cos(angles[mask]), ranges*np.sin(angles[mask])])
        
        plt.figure(figsize=(10, 8))
        for val in cartesian_lasers:
            plt.scatter(val[0], val[1], s=0.5)
        plt.xlabel('X Position (m)')
        plt.ylabel('Y Position (m)')
        if 'circle' in filename:
            plt.title('Laser Scan Data Circle')
        elif 'line' in filename:
            plt.title('Laser Scan Data Line')
        elif 'spiral' in filename:
            plt.title('Laser Scan Data Spiral')
        plt.grid()
        plt.show()
    else:
        for val in values:
            time_list.append(val[-1] - first_stamp)

        num_subplots = len(headers) - 1  # Exclude timestamp column
        fig, axs = plt.subplots(num_subplots, 1, figsize=(10, 3 * num_subplots), sharex=True)
        fig.suptitle(format_title(filename))

        for i in range(num_subplots):
            axs[i].plot(time_list, [lin[i] for lin in values])
            if headers[i] == 'acc_x':
                axs[i].set_ylabel('Acceleration X [m/s^2]')
            elif headers[i] == 'acc_y':
                axs[i].set_ylabel('Acceleration Y [m/s^2]')
            elif headers[i] == 'angular_z':
                axs[i].set_ylabel('Angular Z [rad/s]')
            elif headers[i] == 'x':
                axs[i].set_ylabel('X Position [m]')
            elif headers[i] == 'y':
                axs[i].set_ylabel('Y Position [m]')
            elif headers[i] == 'th':
                axs[i].set_ylabel('Theta [rad]')
            else:
                axs[i].set_ylabel(headers[i])
            axs[i].grid(True)

        axs[-1].set_xlabel('Time (s)')
        plt.show()

import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
 
    args = parser.parse_args()
    
    print("plotting the files", args.files)

    filenames = args.files

    for filename in filenames:
        plot_errors(filename)
