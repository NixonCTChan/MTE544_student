# Imports
import rclpy
from rclpy.node import Node
from utilities import Logger, euler_from_quaternion
from rclpy.qos import QoSProfile

# Importing the necessary message types
from geometry_msgs.msg import Twist  # For sending velocity commands
from sensor_msgs.msg import Imu, LaserScan  # For the sensors
from nav_msgs.msg import Odometry  # For Odometry

from rclpy.time import Time

# Additional imports (if needed)
# import math

CIRCLE = 0
SPIRAL = 1
ACC_LINE = 2
motion_types = ['circle', 'spiral', 'line']

class motion_executioner(Node):
    
    def __init__(self, motion_type=0):
        super().__init__("motion_types")
        
        self.type = motion_type
        self.radius_ = 0.0
        
        self.successful_init = False
        self.imu_initialized = False
        self.odom_initialized = False
        self.laser_initialized = False
        
        # Creating a publisher to send velocity commands
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Loggers
        self.imu_logger = Logger('imu_content_' + str(motion_types[motion_type]) + '.csv', headers=["acc_x", "acc_y", "angular_z", "stamp"])
        self.odom_logger = Logger('odom_content_' + str(motion_types[motion_type]) + '.csv', headers=["x", "y", "th", "stamp"])
        self.laser_logger = Logger('laser_content_' + str(motion_types[motion_type]) + '.csv', headers=["ranges", "angle_increment", "stamp"])
        
        # Creating QoS profile
        qos = QoSProfile(depth=10)
        
        # Subscriptions to sensor topics
        self.create_subscription(Imu, 'imu', self.imu_callback, qos)
        self.create_subscription(Odometry, 'odom', self.odom_callback, qos)
        self.create_subscription(LaserScan, 'scan', self.laser_callback, qos)
        
        # Create a timer that calls the timer_callback every 0.1 seconds
        self.create_timer(0.1, self.timer_callback)

    def imu_callback(self, imu_msg: Imu):
        # Log IMU messages
        acc_x = imu_msg.linear_acceleration.x
        acc_y = imu_msg.linear_acceleration.y
        angular_z = imu_msg.angular_velocity.z
        stamp = Time.from_msg(imu_msg.header.stamp).nanoseconds
        self.imu_logger.log_values([acc_x, acc_y, angular_z, stamp])
        self.imu_initialized = True

    def odom_callback(self, odom_msg: Odometry):
        # Log odometry messages
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        orientation = odom_msg.pose.pose.orientation
        _, _, th = euler_from_quaternion(orientation)
        stamp = Time.from_msg(odom_msg.header.stamp).nanoseconds
        self.odom_logger.log_values([x, y, th, stamp])
        self.odom_initialized = True

    def laser_callback(self, laser_msg: LaserScan):
        # Log laser scan messages
        ranges = laser_msg.ranges
        angle_increment = laser_msg.angle_increment
        stamp = Time.from_msg(laser_msg.header.stamp).nanoseconds
        self.laser_logger.log_values([ranges, angle_increment, stamp])
        self.laser_initialized = True

    def timer_callback(self):
        if self.odom_initialized and self.laser_initialized and self.imu_initialized:
            self.successful_init = True
            
        if not self.successful_init:
            return
        
        cmd_vel_msg = Twist()
        
        if self.type == CIRCLE:
            cmd_vel_msg = self.make_circular_twist()
        
        elif self.type == SPIRAL:
            cmd_vel_msg = self.make_spiral_twist()
                        
        elif self.type == ACC_LINE:
            cmd_vel_msg = self.make_acc_line_twist()
            
        else:
            print("type not set successfully, 0: CIRCLE 1: SPIRAL and 2: ACCELERATED LINE")
            raise SystemExit 

        self.vel_publisher.publish(cmd_vel_msg)

    def make_circular_twist(self):
        msg = Twist()
        msg.linear.x = 0.2  # Constant forward speed
        msg.angular.z = 0.2  # Constant angular speed for circular motion
        return msg

    def make_spiral_twist(self):
        self.radius_ += 0.01  # Increase radius for spiral motion
        msg = Twist()
        msg.linear.x = self.radius_ * 0.2  # Forward speed proportional to radius
        msg.angular.z = 0.2  # Constant angular speed
        return msg

    def make_acc_line_twist(self):
        msg = Twist()
        msg.linear.x = 0.5  # Constant acceleration in a straight line
        return msg

import argparse

if __name__ == "__main__":
    argParser = argparse.ArgumentParser(description="input the motion type")
    argParser.add_argument("--motion", type=str, default="circle")
    
    rclpy.init()

    args = argParser.parse_args()

    if args.motion.lower() == "circle":
        ME = motion_executioner(motion_type=CIRCLE)
    elif args.motion.lower() == "line":
        ME = motion_executioner(motion_type=ACC_LINE)
    elif args.motion.lower() == "spiral":
        ME = motion_executioner(motion_type=SPIRAL)
    else:
        print(f"we don't have {args.motion.lower()} motion type")
    
    try:
        rclpy.spin(ME)
    except KeyboardInterrupt:
        print("Exiting")
