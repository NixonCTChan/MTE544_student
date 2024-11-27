from planner import TRAJECTORY_PLANNER, planner
import matplotlib.pyplot as plt
import numpy as np


def visualize_path(map_obj, path):
    plt.figure(figsize=(10, 8))
    plt.imshow(map_obj, cmap='binary')
    
    if path:
        path_array = np.array(path)
        plt.plot(path_array[:, 0], path_array[:, 1], 'r-o')
    
    plt.title('Path Planning Visualization')
    plt.xlabel('X coordinate')
    plt.ylabel('Y coordinate')
    plt.show()

# Main execution
if __name__ == "__main__":
    # Create a trajectory planner
    trajectory_planner = planner(TRAJECTORY_PLANNER)
    
    # Define start and end poses (cartesian coordinates)
    start_pose = [5, 5]   # Starting point
    end_pose = [45, 45]   # Destination point
    
    # Plan trajectory
    path = trajectory_planner.plan(start_pose, end_pose)
    
    # Print the path
    print("Planned Path:")
    for i, pose in enumerate(path):
        print(f"Step {i}: {pose}")
    
    # Visualize the path
    visualize_path(trajectory_planner.costMap, path)