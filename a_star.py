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


def search(maze, start, end, heuristic='euclidean'):
    print(f"Searching using {heuristic} heuristic...")

    maze = maze.T

    # Select heuristic function
    if heuristic == 'manhattan':
        heuristic_func = manhattan_distance
    else:
        heuristic_func = euclidean_distance

    # Create start and end nodes
    start_node = Node(None, start)
    start_node.g = 0
    start_node.h = heuristic_func(start, end)
    start_node.f = start_node.h

    end_node = Node(None, end)
    end_node.g = 100
    end_node.h = 0
    end_node.f = end_node.g + end_node.h

    # Initialize dictionaries for nodes to visit and visited nodes
    yet_to_visit_dict = {}
    visited_dict = {}

    yet_to_visit_dict[start_node.position] = start_node

    # Iteration control
    outer_iterations = 0
    max_iterations = (len(maze) // 2) ** 10

    # Define movement possibilities (8-directional)
    move = [[-1, 0],   # go up
            [0, -1],   # go left
            [1, 0],    # go down
            [0, 1],    # go right
            [-1, -1],  # go up left
            [1, -1],   # go down left
            [-1, 1],   # go up right
            [1, 1]]    # go down right

    # Get maze dimensions
    no_rows, no_columns = maze.shape

    # Main search loop
    while len(yet_to_visit_dict) > 0:
        outer_iterations += 1

        # Find node with lowest f score
        current_node = min(yet_to_visit_dict.values(), key=lambda node: node.f)

        # Check iteration limit
        if outer_iterations > max_iterations:
            print("giving up on pathfinding too many iterations")
            return return_path(current_node, maze)

        # Move current node from yet_to_visit to visited
        yet_to_visit_dict.pop(current_node.position)
        visited_dict[current_node.position] = True

        # Check if goal is reached
        if current_node == end_node:
            return return_path(current_node, maze)

        # Generate children nodes
        children = []

        for new_position in move:
            # Calculate new node position
            node_position = (current_node.position[0] + new_position[0], 
                             current_node.position[1] + new_position[1])

            # Check maze boundaries
            if (node_position[0] < 0 or 
                node_position[0] >= no_rows or 
                node_position[1] < 0 or 
                node_position[1] >= no_columns):
                continue

            # Check for obstacles
            if maze[node_position[0], node_position[1]] > 0.8:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Calculate movement cost
            if (new_position[0] != 0 and new_position[1] != 0):
                # Diagonal movement
                movement_cost = sqrt(2)
            else:
                # Orthogonal movement
                movement_cost = 1

            # Calculate node costs
            new_node.g = current_node.g + movement_cost
            new_node.h = heuristic_func(node_position, end)
            new_node.f = new_node.g + new_node.h

            # Skip if already visited
            if visited_dict.get(new_node.position, False):
                continue

            # Check if node is already in yet_to_visit with lower g-score
            existing_node = yet_to_visit_dict.get(new_node.position)
            if existing_node:
                if new_node.g >= existing_node.g:
                    continue
                else:
                    # Replace existing node if new path is better
                    yet_to_visit_dict[new_node.position] = new_node
            else:
                # Add new node to yet_to_visit
                yet_to_visit_dict[new_node.position] = new_node

    # No path found
    return None