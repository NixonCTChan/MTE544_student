import sys

from utilities import Logger

from rclpy.time import Time

from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error
from rclpy.node import Node
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from sensor_msgs.msg import Imu
from kalman_filter import kalman_filter

from rclpy import init, spin, spin_once

import numpy as np
import message_filters

rawSensors=0
kalmanFilter=1
odom_qos=QoSProfile(reliability=2, durability=2, history=1, depth=10)

class localization(Node):
    
    def __init__(self, type, dt, loggerName="robotPose.csv", loggerHeaders=["imu_ax", "imu_ay", "kf_ax", "kf_ay","kf_vx","kf_w","kf_x", "kf_y","stamp"]):

        super().__init__("localizer")

        self.loc_logger=Logger( loggerName , loggerHeaders)
        self.pose=None
        
        if type==rawSensors:
            self.initRawSensors()
        elif type==kalmanFilter:
            self.initKalmanfilter(dt)
        else:
            print("We don't have this type for localization", sys.stderr)
            return  

    def initRawSensors(self):
        self.create_subscription(odom, "/odom", self.odom_callback, qos_profile=odom_qos)
        
    def initKalmanfilter(self, dt):
        
        # Initial state vector [x, y, th, w, v, vdot]
        x = np.zeros(6)
        
        # Process noise covariance
        Q = np.diag([
            0.5,    # x position
            0.5,    # y position
            0.5,    # heading
            0.5,    # angular velocity
            0.5,    # linear velocity
            0.5     # acceleration
        ])
        
        # Measurement noise covariance
        R = np.diag([
            0.5,    # velocity measurement
            0.5,    # angular velocity measurement
            0.5,    # ax measurement
            0.5     # ay measurement
        ])
        
        # Initial state covariance
        P = np.eye(6) * 1.0
        
        self.kf = kalman_filter(P, Q, R, x, dt)
        
        # Set up subscribers with message filters
        self.odom_sub = message_filters.Subscriber(
            self, odom, "/odom", qos_profile=odom_qos)
        self.imu_sub = message_filters.Subscriber(
            self, Imu, "/imu", qos_profile=odom_qos)
        
        time_syncher=message_filters.ApproximateTimeSynchronizer([self.odom_sub, self.imu_sub], queue_size=10, slop=0.1)
        time_syncher.registerCallback(self.fusion_callback)
    
    def fusion_callback(self, odom_msg: odom, imu_msg: Imu):
        # Extract measurements from sensors
        z = np.array([
            odom_msg.twist.twist.linear.x,     # v
            odom_msg.twist.twist.angular.z,    # w
            imu_msg.linear_acceleration.x,     # ax
            imu_msg.linear_acceleration.y      # ay
        ])
        
        # Perform EKF prediction and update steps
        self.kf.predict()
        self.kf.update(z)
        
        # Get updated state estimate
        xhat = self.kf.get_states()
        
        # Update pose for getPose method
        self.pose = [
            xhat[0],  # x
            xhat[1],  # y
            xhat[2],  # th
            imu_msg.header.stamp
        ]
        
        # Log data
        self.loc_logger.log_values([
            imu_msg.linear_acceleration.x,  # imu_ax
            imu_msg.linear_acceleration.y,  # imu_ay
            xhat[5],                       # kf_ax
            xhat[3] * xhat[4],            # kf_ay
            xhat[4],                       # kf_vx
            xhat[3],                       # kf_w
            xhat[0],                       # kf_x
            xhat[1],                       # kf_y
            imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec * 1e-9  # stamp
        ])
      
    def odom_callback(self, pose_msg):
        
        self.pose=[ pose_msg.pose.pose.position.x,
                    pose_msg.pose.pose.position.y,
                    euler_from_quaternion(pose_msg.pose.pose.orientation),
                    pose_msg.header.stamp]

    # Return the estimated pose
    def getPose(self):
        return self.pose


if __name__=="__main__":
    
    init()
    
    LOCALIZER=localization()
    
    spin(LOCALIZER)
