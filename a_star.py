import numpy as np
from math import sqrt


class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


def manhattan_distance(point1, point2):
    """Calculate Manhattan (L1) distance between two points"""
    return abs(point1[0] - point2[0]) + abs(point1[1] - point2[1])


def euclidean_distance(point1, point2):
    """Calculate Euclidean (L2) distance between two points"""
    return sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)


def return_path(current_node, maze):
    """Reconstruct and return the path from start to end"""
    path = []
    no_rows, no_columns = np.shape(maze)
    result = [[-1 for i in range(no_columns)] for j in range(no_rows)]
    current = current_node
    while current is not None:
        path.append(current.position)
        current = current.parent
    
    path = path[::-1]
    start_value = 0
    for i in range(len(path)):
        result[path[i][0]][path[i][1]] = start_value
        start_value += 1

    return path


def search(maze, start, end, heuristic='manhattan'):
    print(f"Searching using {heuristic} heuristic...")
    print(f"Maze shape: {maze.shape}")
    
    maze = maze.T
    print(f"Transposed maze shape: {maze.shape}")
    
    # Select heuristic function
    if heuristic == 'manhattan':
        heuristic_func = manhattan_distance
        print("Using Manhattan distance heuristic")
    else:
        heuristic_func = euclidean_distance
        print("Using Euclidean distance heuristic")
    
    # Create start and end nodes
    start_node = Node(None, start)
    start_node.g = 0
    start_node.h = heuristic_func(start, end)
    start_node.f = start_node.h
    print(f"Start Node - Position: {start_node.position}, g: {start_node.g}, h: {start_node.h}, f: {start_node.f}")
    
    end_node = Node(None, end)
    end_node.g = 1
    end_node.h = 0
    end_node.f = end_node.g + end_node.h
    print(f"End Node - Position: {end_node.position}, g: {end_node.g}, h: {end_node.h}, f: {end_node.f}")
    
    # Initialize dictionaries for nodes to visit and visited nodes
    yet_to_visit_dict = {}
    visited_dict = {}
    yet_to_visit_dict[start_node.position] = start_node
    print("Initial yet_to_visit_dict initialized")
    
    # Iteration control
    outer_iterations = 0
    max_iterations = (len(maze) // 2) ** 10
    print(f"Max iterations set to: {max_iterations}")
    
    # Define movement possibilities (8-directional)
    move = [
        [-1, 0],   # go up
        [0, -1],   # go left
        [1, 0],    # go down
        [0, 1],    # go right
        [-1, -1],  # go up left
        [1, -1],   # go down left
        [-1, 1],   # go up right
        [1, 1]     # go down right
    ]
    print("Movement directions defined")
    
    # Get maze dimensions
    no_rows, no_columns = maze.shape
    print(f"Maze dimensions: {no_rows} rows, {no_columns} columns")
    # Main search loop
    while len(yet_to_visit_dict) > 0:
        outer_iterations += 1
        print(f"\n--- Iteration {outer_iterations} ---")
        print(f"Nodes yet to visit: {len(yet_to_visit_dict)}")
        
        # Find node with lowest f score
        current_node = min(yet_to_visit_dict.values(), key=lambda node: node.f)
        print(f"Current node selected: {current_node.position}, f: {current_node.f}")
        
        # Check iteration limit
        if outer_iterations > max_iterations:
            print("Giving up on pathfinding - too many iterations")
            return return_path(current_node, maze)
        
        # Move current node from yet_to_visit to visited
        yet_to_visit_dict.pop(current_node.position)
        visited_dict[current_node.position] = True
        print(f"Added {current_node.position} to visited nodes")
        # print(f"Visited nodes: {list(visited_dict.keys())}")
        
        # Check if goal is reached
        if current_node.position == end_node.position:
            print("Goal reached!")
            return return_path(current_node, maze)
        
        # Generate children nodes
        print("Exploring neighboring nodes:")
        for new_position in move:
            # Calculate new node position
            node_position = (
                current_node.position[0] + new_position[0],
                current_node.position[1] + new_position[1]
            )
            print(f"  Checking potential neighbor: {node_position}")
            
            # Check maze boundaries
            if (node_position[0] < 0 or 
                node_position[0] >= no_rows or 
                node_position[1] < 0 or 
                node_position[1] >= no_columns):
                print(f"    Skipping {node_position} - out of bounds")
                continue
            
            # Check for obstacles
            if maze[node_position[0], node_position[1]] > 0.65:
                print(f"    Skipping {node_position} - obstacle detected value:{maze[node_position[0], node_position[1]]}")
                continue
            # Create new node
            new_node = Node(current_node, node_position)
            
            # Calculate movement cost
            if (new_position[0] != 0 and new_position[1] != 0):
                # Diagonal movement
                movement_cost = sqrt(2)
                print(f"    Diagonal movement to {node_position}, cost: {movement_cost}")
            else:
                # Orthogonal movement
                movement_cost = 1
                print(f"    Orthogonal movement to {node_position}, cost: {movement_cost}")
            
            # Calculate node costs
            new_node.g = current_node.g + movement_cost
            new_node.h = heuristic_func(node_position, end)
            new_node.f = new_node.g + new_node.h
            print(f"    Node costs - g: {new_node.g}, h: {new_node.h}, f: {new_node.f}")
            
            # Skip if already visited
            if visited_dict.get(new_node.position, False):
                print(f"    Skipping {node_position} - already visited")
                continue
            
            # Check if node is already in yet_to_visit with lower g-score
            existing_node = yet_to_visit_dict.get(new_node.position)
            if existing_node:
                if new_node.g >= existing_node.g:
                    print(f"    Skipping {node_position} - existing path is better")
                    continue
                else:
                    # Replace existing node if new path is better
                    yet_to_visit_dict[new_node.position] = new_node
                    print(f"    Updating existing node {node_position} with better path")
            else:
                # Add new node to yet_to_visit
                yet_to_visit_dict[new_node.position] = new_node
                print(f"    Added {node_position} to nodes to visit")
    
    # No path found
    print("No path found")
    return None

import matplotlib.pyplot as plt
import matplotlib.pyplot as plt
import numpy as np

def plot_maze_with_path(maze, path=None):
    """
    Plot a maze where color intensity corresponds to position values,
    with optional path overlay.
    
    Parameters:
    maze (numpy.ndarray): 2D numpy array with values between 0 and 1
    path (list of tuples, optional): List of (x,y) coordinates forming the path
    """
    # Create a figure and axis
    plt.figure(figsize=(10, 8))
    plt.imshow(maze, cmap='viridis', alpha=maze)
    
    # Add a colorbar
    plt.colorbar(label='Maze Position Values')
    
    # Plot the path if provided
    if path is not None and len(path) > 1:
        # Extract x and y coordinates
        x_coords = [p[0] for p in path]  # Note the swapped indices
        y_coords = [p[1] for p in path]  # Swap to match imshow's origin='lower'
        
        # Plot the path as a red line with markers
        plt.plot(x_coords, y_coords, color='red', linewidth=2, 
                 marker='o', markersize=6, 
                 markerfacecolor='white', markeredgecolor='red')
        
        # Mark start and end points differently
        plt.plot(x_coords[0], y_coords[0], color='green', marker='o', 
                 markersize=10, markerfacecolor='white', markeredgecolor='green', 
                 label='Start')
        plt.plot(x_coords[-1], y_coords[-1], color='blue', marker='o', 
                 markersize=10, markerfacecolor='white', markeredgecolor='blue', 
                 label='End')
    
    # Set title and labels
    plt.title('Maze Visualization with Path')
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    
    # Show grid lines
    plt.grid(color='white', linestyle='-', linewidth=0.5)
    
    # Add legend if path is plotted
    if path is not None:
        plt.legend()
    
    # Tight layout to prevent cutting off labels
    plt.tight_layout()
    
    # Show the plot
    plt.show()