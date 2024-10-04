# You can use this file to plot the loged sensor data
# Note that you need to modify/adapt it to your own files
# Feel free to make any modifications/additions here

import matplotlib.pyplot as plt
import numpy as np
from utilities import FileReader

def plot_errors(filename):
    
    headers, values=FileReader(filename).read_file() 
    time_list=[]
    first_stamp=values[0][-1]
    
    if "laser" in filename:
        content_headers, content_values=FileReader(filename.replace("laser_ranges_content", "laser_content")).read_file() 
        ranges_headers, ranges_values=FileReader(filename.replace("laser_content", "laser_ranges_content")).read_file()
        time_list=[]
        cartesian_lasers=[]
        first_stamp=content_values[0][-1]
        angle = content_values[0][0]
        angles = np.linspace(0, angle * len(ranges_values[0][0]), len(ranges_values[0][0]))
        angles = np.array([angles])
        for val in zip(content_values, ranges_values):
            time_list.append(val[0][1] - first_stamp)
            ranges = np.array(val[1])
            mask = ranges != np.inf
            ranges = ranges[mask]
            cartesian_lasers.append([ranges*np.cos(angles[mask]), ranges*np.sin(angles[mask])])
        for val in cartesian_lasers:
            plt.scatter(val[0], val[1], s=0.5) 
        plt.grid()
        plt.show()   
    else:
        for val in values:
            time_list.append(val[-1] - first_stamp)

        for i in range(0, len(headers) - 1):
            plt.plot(time_list, [lin[i] for lin in values], label= headers[i]+ " linear")
        plt.legend()
        plt.grid()
        plt.show()
        
import argparse

if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
    
    args = parser.parse_args()
    
    print("plotting the files", args.files)

    filenames=args.files
    for filename in filenames:
        plot_errors(filename)
