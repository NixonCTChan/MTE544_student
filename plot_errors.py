import matplotlib.pyplot as plt
from utilities import FileReader
import numpy as np




def plot_errors(filename):
    
    headers, values=FileReader(filename).read_file()
    
    time_list=[]
    
    first_stamp=values[0][-1]
    
    for val in values:
        time_list.append(val[-1] - first_stamp)

    
    
    fig, axes = plt.subplots(2,1, figsize=(14,15))

    # Example timestamps (you can replace this with your actual list of timestamps)
    timestamps = [lin[len(headers) - 1] for lin in values] # Replace with actual timestamps

    # Initial conditions
    linear_velocity = 0.0
    angular_velocity = 1.0  # constant angular velocity in rad/s
    x_position = 0.0
    y_position = 0.0
    orientation = 0.0  # Initial orientation angle in radians

    # Lists to store x and y positions for plotting
    x_positions = [x_position]
    y_positions = [y_position]
    gt_time_list = [0]
    # Time-step loop
    for i in range(1, len(timestamps)):
        dt = 1/10
        gt_time_list.append(gt_time_list[i-1] + dt)
        
        # Update linear velocity
        linear_velocity += 0.01 if linear_velocity < 1.0 else 0.0
        
        # Update orientation based on angular velocity
        orientation += angular_velocity * dt
        
        # Update x and y positions based on current velocity and orientation
        x_position += linear_velocity * np.cos(orientation) * dt
        y_position += linear_velocity * np.sin(orientation) * dt
        
        # Store the positions
        x_positions.append(x_position)
        y_positions.append(y_position)

    # Plot 2D "state space" (trajectory)
    # axes[0].plot(x_positions, y_positions, label="GT")
    axes[0].set_title("State Space (2D Trajectory)")
    axes[0].set_xlabel("X Position (m)")
    axes[0].set_ylabel("Y Position (m)")

    axes[0].plot([lin[len(headers) - 3] for lin in values], [lin[len(headers) - 2] for lin in values], label="KF xy")
    axes[0].grid()
    axes[0].legend()
    axes[0].set_aspect('equal')

    
    axes[1].set_title("each individual state")
    for i in range(0, len(headers) - 1):
        if "x" not in headers[i] and "y" not in headers[i]:
             continue
        axes[1].plot(time_list, [lin[i] for lin in values], label= headers[i])
    axes[1].plot(gt_time_list, x_positions, label="gt_x")
    axes[1].plot(gt_time_list, y_positions, label="gt_y")

    axes[1].legend()
    axes[1].grid()


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