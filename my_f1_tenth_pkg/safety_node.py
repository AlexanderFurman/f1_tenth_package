#!/usr/bin/env python
import rclpy 
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from math import isfinite, cos, sin

import numpy as np

# TODO: import ROS msg types and libraries

class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        """
        One publisher should publish to the /brake topic with a AckermannDriveStamped brake message.

        One publisher should publish to the /brake_bool topic with a Bool message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        super().__init__('safety_node')
        self.speed = 0
        self.odom_sub_ = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.scan_sub_ = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.drive_pub_ = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.epsilon = 0.01
        
    def odom_callback(self, odom_msg):
        self.speed = odom_msg.twist.twist

    def scan_callback(self, scan_msg):
        # TODO: calculate TTC
        TTC_threshold = 2
        min_TTC = 50
        vel_x = self.speed.linear.x
        vel_y = self.speed.linear.y
        angle_min = scan_msg.angle_min
        angle_max = scan_msg.angle_max
        increment = scan_msg.angle_increment
        ranges = np.array(scan_msg.ranges)

        for i in range(len(ranges)):
            #check not nan and not inf
            if isfinite(ranges[i]):
                local_dist = ranges[i]
                local_angle = angle_min + increment*i
                local_derivative = vel_x*cos(local_angle) + vel_y*sin(local_angle)

                try:
                    local_TTC = local_dist/local_derivative
                except ZeroDivisionError:
                    local_TTC = 0 #min_TTC
                
                if (local_derivative > 0) and (local_TTC < min_TTC):
                    min_TTC = local_TTC
            if min_TTC <= TTC_threshold:
                vel = AckermannDriveStamped()
                vel.drive.speed = float(0)
                self.drive_pub_.publish(vel)
                print("EMERGENCY BRAKING PROCEDURE INITIATED")

def main(args=None):
    print("safety node initiated")
    rclpy.init(args=args)
    sn = SafetyNode()
    rclpy.spin(sn)
    safety_node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()