import numpy as np


from pid import PID_ctrl
from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error

M_PI=3.1415926535

P=0; PD=1; PI=2; PID=3

class controller:
    
    # Default gains of the controller for linear and angular motions
    def __init__(self, klp=0.2, klv=0.2, kli=0.2, kap=0.2, kav=0.2, kai=0.2):
        
        # TODO Part 5 and 6: Modify the below lines to test your PD, PI, and PID controller

        # CHANGE FOR EACH TEST: P=0; PD=1; PI=2; PID=3
        controller_choice = P

        if controller_choice == P:
            self.PID_linear = PID_ctrl(P, klp, klv, kli, filename_="P_linear.csv")
            self.PID_angular = PID_ctrl(P, kap, kav, kai, filename_="p_angular.csv")
        elif controller_choice == PD:
            self.PID_linear = PID_ctrl(PD, klp, klv, kli, filename_="PD_linear.csv")
            self.PID_angular = PID_ctrl(PD, kap, kav, kai, filename_="PD_angular.csv")
        elif controller_choice == PI:
            self.PID_linear = PID_ctrl(PI, klp, klv, kli, filename_="PI_linear.csv")
            self.PID_angular = PID_ctrl(PI, kap, kav, kai, filename_="PI_angular.csv")
        elif controller_choice == PID:
            self.PID_linear = PID_ctrl(PID, klp, klv, kli, filename_="PID_linear.csv")
            self.PID_angular = PID_ctrl(PID, kap, kav, kai, filename_="PID_angular.csv")

    
    def vel_request(self, pose, goal, status):
        
        # From utilities.py
        e_lin=calculate_linear_error(pose, goal)
        e_ang=calculate_angular_error(pose, goal)

        linear_vel=self.PID_linear.update([e_lin, pose[3]], status)
        angular_vel=self.PID_angular.update([e_ang, pose[3]], status)
        
        # TODO Part 4: Add saturation limits for the robot linear and angular velocity
        # FIXME CHANGE FOR IN PERSON!
        # FOR SIMU
        max_linear_vel = 0.22  # Define the maximum linear velocity
        max_angular_vel = 2.84  # Define the maximum angular velocity
        # FOR IN PERSON
        #max_linear_vel = 0.31  # Define the maximum linear velocity
        #max_angular_vel = 1.90  # Define the maximum angular velocity

        linear_vel = max_linear_vel if linear_vel > max_linear_vel else linear_vel
        angular_vel= max_angular_vel if angular_vel > max_angular_vel else angular_vel
        
        return linear_vel, angular_vel
    

class trajectoryController(controller):

    def __init__(self, klp=0.2, klv=0.2, kli=0.2, kap=0.2, kav=0.2, kai=0.2):
        
        super().__init__(klp, klv, kli, kap, kav, kai)
    
    def vel_request(self, pose, listGoals, status):
        
        goal=self.lookFarFor(pose, listGoals)
        
        finalGoal=listGoals[-1]
        
        e_lin=calculate_linear_error(pose, finalGoal)
        e_ang=calculate_angular_error(pose, goal)

        
        linear_vel=self.PID_linear.update([e_lin, pose[3]], status)
        angular_vel=self.PID_angular.update([e_ang, pose[3]], status) 

        # TODO Part 5: Add saturation limits for the robot linear and angular velocity
        # FIXME CHANGE FOR IN PERSON!
        # FOR SIMU
        max_linear_vel = 0.22  # Define the maximum linear velocity
        max_angular_vel = 2.84  # Define the maximum angular velocity
        # FOR IN PERSON
        #max_linear_vel = 0.31  # Define the maximum linear velocity
        #max_angular_vel = 1.90  # Define the maximum angular velocity

        linear_vel = max_linear_vel if linear_vel > max_linear_vel else linear_vel
        angular_vel= max_angular_vel if angular_vel > max_angular_vel else angular_vel
        
        return linear_vel, angular_vel

    def lookFarFor(self, pose, listGoals):
        
        poseArray=np.array([pose[0], pose[1]]) 
        listGoalsArray=np.array(listGoals)

        distanceSquared=np.sum((listGoalsArray-poseArray)**2,
                               axis=1)
        closestIndex=np.argmin(distanceSquared)

        return listGoals[ min(closestIndex + 3, len(listGoals) - 1) ]