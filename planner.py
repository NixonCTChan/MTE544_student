from mapUtilities import *
from a_star import *

POINT_PLANNER=0; TRAJECTORY_PLANNER=1

class planner:
    def __init__(self, type_, mapName="room"):

        self.type=type_
        self.mapName=mapName

    
    def plan(self, startPose, endPose):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(endPose)
        
        elif self.type==TRAJECTORY_PLANNER:
            self.costMap=None
            self.initTrajectoryPlanner()
            return self.trajectory_planner(startPose, endPose)


    def point_planner(self, endPose):
        return endPose

    def initTrajectoryPlanner(self):
        # Create the cost-map with a standard deviation for the Gaussian
        # Using a reasonable default standard deviation (e.g., 1.0)
        self.m_utilites = mapManipulator(laser_sig=1.0)
            
        self.costMap = self.m_utilites.make_likelihood_field()
        

    def trajectory_planner(self, startPoseCart, endPoseCart):
        # Convert cartesian coordinates to pixel coordinates
        startPose = self.m_utilites.position_2_cell(startPoseCart)
        endPose = self.m_utilites.position_2_cell(endPoseCart)
        
        # Use A* search to find the path in the cost map
        # Using Euclidean heuristic for more natural path
        Path = search(self.costMap, startPose, endPose, heuristic='euclidean')
        
        # Convert pixel coordinates back to cartesian coordinates
        # Convert each pixel coordinate to its cartesian representation
        cartesianPath = list(map(self.m_utilites.cell_2_position, Path))

        # Return path as list of [x,y] in cartesian coordinates
        return cartesianPath


if __name__=="__main__":
    # Example usage for testing
    m_utilites = mapManipulator()
    
    map_likelihood = m_utilites.make_likelihood_field()

    # You can use this part of the code to test your 
    # search algorithm regardless of the ROS2 complexities

    # Create a trajectory planner
    trajectory_planner = planner(TRAJECTORY_PLANNER)
    
    # Define start and end poses (cartesian coordinates)
    start_pose = [0, 0]  # Example start point
    end_pose = [10, 10]  # Example end point
    
    # Plan trajectory
    path = trajectory_planner.plan(start_pose, end_pose)
    
    print("Planned Path:", path)