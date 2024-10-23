# Type of planner
import math
POINT_PLANNER=0; TRAJECTORY_PLANNER=1



class planner:
    def __init__(self, type_):

        self.type=type_

    
    def plan(self, goalPoint=[-1.0, -1.0]):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(goalPoint)
        
        elif self.type==TRAJECTORY_PLANNER:
            return self.trajectory_planner()


    def point_planner(self, goalPoint):
        x = goalPoint[0]
        y = goalPoint[1]
        return x, y

    # TODO Part 6: Implement the trajectories here
    def trajectory_planner(self):

        # FIXME Choose
        # Option 0: y = x^2
        # Option 1: sigma = 2 / (1+e^(-2x)) - 1
        traj_choice = 1

        trajectory_points = []

        # y = x^2
        # If increments of 0.1 in range [0.0, 1.5]
        if traj_choice == 0:
            for x in range(0, 16):
                x_val = x / 10.0
                y_val = x_val ** 2
                trajectory_points.append([x_val, y_val])

        # sigma = 2 / (1+e^(-2x)) - 1
        # If increments of 0.1 in range [0.0, 2.5]
        elif traj_choice == 1:
            for x in range(0, 26):  
                x_val = x / 10.0  # Convert to float in the range [0.0, 2.5]
                y_val = 2 / (1 + math.exp(-2 * x_val)) - 1
                trajectory_points.append([x_val, y_val])

        # the return should be a list of trajectory points: [ [x1,y1], ..., [xn,yn]]
        return trajectory_points