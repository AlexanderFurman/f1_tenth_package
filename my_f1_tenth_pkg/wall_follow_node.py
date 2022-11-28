

import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from math import sin, cos, isfinite
import time


class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: create subscribers and publishers
        self.time_current = 0
        self.time_previous = 0

        self.following_distance = 1 #requested distance to wall
        self.lookahead_param = 1 #how much our car is projected to move forward between servo commands
        # self.odom_sub_ = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.scan_sub_ = self.create_subscription(LaserScan, lidarscan_topic, self.scan_callback, 10)
        self.drive_pub_ = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

        # TODO: set PID gains
        self.kp = 2
        self.kd = 2
        self.ki = 1

        # TODO: store history
        self.integral = 0
        self.prev_error = 0
        self.error = 0

        # TODO: store any necessary values you think you'll need

    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.
        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR
        Returns:
            range: range measurement in meters at the given angle
        """

        #TODO: implement

        angle_min = -2.3499999046325684
        angle_max = 2.3499999046325684
        angle_increment = 0.004351851996034384
        
        index = int(np.floor((angle-angle_min)/angle_increment))
        try:
            return range_data[index]

        except IndexError as e:
            print(f"{e}")
            print(f"length of dists = {len(range_data)}, and you're asking for index {index}!")

        return None

    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()
        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall
        Returns:
            error: calculated error
        """

        # transform = 90
        # angle2 = np.deg2rad(-70)
        # angle1 = np.deg2rad(-90)
        angle_b = np.deg2rad(90 - 0)
        angle_a = np.deg2rad(90 - 45)

        while (not isfinite(self.get_range(range_data, angle_a))) or (angle_a == angle_b):
            angle_a += 0.01

        while (not isfinite(self.get_range(range_data, angle_b))) or (angle_a == angle_b):
            angle_b += 0.01

        theta = -(angle_a-angle_b)
        range_a = self.get_range(range_data, angle_a)
        range_b = self.get_range(range_data, angle_b)

        num = range_a*np.cos(theta) - range_b
        dem = range_a*np.sin(theta)

        alpha = np.arctan2(num,dem)
        D_t = range_b*np.cos(alpha)
        
        L = self.lookahead_param
        D_t_plus_1 = D_t + L*sin(alpha)

        error = self.following_distance - D_t_plus_1
        self.prev_error = self.error
        self.error = error
        self.time_previous = self.time_current
        self.time_current = time.time()
        time_diff = self.time_current - self.time_previous
        self.integral = self.prev_error*(time_diff)

        return error

    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control
        Args:
            error: calculated error
            velocity: desired velocity
        Returns:
            None
        """
        angle = 0.0
        angle = -(self.kp*error + self.ki*self.integral + self.kd*(error-self.prev_error)/(self.time_current-self.time_previous))
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = angle

        if abs(np.rad2deg(angle)) < 10:
            velocity = 3. #1.5
        elif (abs(np.rad2deg(angle)) > 10 and abs(np.rad2deg(angle)) < 20):
            velocity = 1.
        else:
            velocity = 0.5
        
        drive_msg.drive.speed = velocity

        self.drive_pub_.publish(drive_msg)

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.
        Args:
            msg: Incoming LaserScan message
        Returns:
            None
        """
        error = self.get_error(msg.ranges, self.following_distance) # TODO: replace with error calculated by get_error()
        velocity = 0.0 # TODO: calculate desired car velocity based on error
        self.pid_control(error, velocity) # TODO: actuate the car with PID


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()