import matplotlib.pyplot as plt
from utilities import FileReader

def plot_errors():
    # Read the CSV file
    headers, values = FileReader("Poses/euclidean/Try_01.csv").read_file()
    
    # Extract timestamp and convert to relative time
    timestamps = [float(val[-1]) for val in values]
    first_stamp = timestamps[0]
    time_list = [t - first_stamp for t in timestamps]
    
    # Extract x, y, theta values
    x_vals = [val[0] for val in values]
    y_vals = [val[1] for val in values]
    theta_vals = [val[2] for val in values]
    
    # Create a figure with two subplots
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10))
    
    # Plot 1: Absolute Position Path
    ax1.plot(x_vals, y_vals)
    ax1.set_title('Robot Absolute Position Path')
    ax1.set_xlabel('X Position')
    ax1.set_ylabel('Y Position')
    ax1.grid(True)
    ax1.axis('equal')  # Ensure equal scaling of x and y axes
    
    # Plot 2: Values Across Time
    ax2.plot(time_list, x_vals, label='X Position', marker='.')
    ax2.plot(time_list, y_vals, label='Y Position', marker='.')
    ax2.plot(time_list, theta_vals, label='Theta', marker='.')
    
    ax2.set_title('Robot Pose Values Over Time')
    ax2.set_xlabel('Time (nanoseconds)')
    ax2.set_ylabel('Values')
    ax2.legend()
    ax2.grid(True)
    
    # Adjust layout and show plot
    plt.tight_layout()
    plt.show()
    

if __name__=="__main__":
    plot_errors()