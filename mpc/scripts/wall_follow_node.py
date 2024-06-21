#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # create subscribers and publishers
        self.scan_subscriber = self.create_subscription(LaserScan, lidarscan_topic, self.scan_callback, 10)
        self.ackermann_publisher = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

        # set PID gains
        self.kp = 1.0
        self.kd = 0.001
        self.ki = 0.005

        # store history
        self.integral = 0.0
        self.prev_error = 0.0
        self.error = 0.0

        # store any necessary values you think you'll need
        self.prev_t = self.get_clock().now().nanoseconds / 1e9
        self.del_t = 0.0
        self.speed = 0.0
        self.lookahead = 1.0
        self.desired_distance = 1.1

        #flag to toggle between left and right wall follow. 1: left wall, -1: right wall 
        self.flag = 1

    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """

        ind = int((angle - range_data.angle_min) / range_data.angle_increment)
        if not np.isnan(range_data.ranges[ind]) and not np.isinf(range_data.ranges[ind]):
            return range_data.ranges[ind]
        else:
            return 0.0

    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """

        a_angle = self.flag * 45.0 * np.pi / 180.0 
        b_angle = self.flag * 90.0 * np.pi / 180.0

        a = self.get_range(range_data, a_angle)
        b = self.get_range(range_data, b_angle)

        theta = b_angle - a_angle

        alpha = np.arctan((a * np.cos(theta) - b) / (a * np.sin(theta)))
        d_t = b * np.cos(alpha)
        d_t1 = d_t + self.lookahead * np.sin(alpha)

        return dist - d_t1

    def pid_control(self):
        """
        Based on the calculated error, publish vehicle control

        Returns:
            None
        """
        t = self.get_clock().now().nanoseconds / 1e9
        self.del_t = t - self.prev_t
        self.integral += self.prev_error * self.del_t
        self.prev_t = t

        # Use kp, ki & kd to implement a PID controller
        angle = self.flag * -(self.kp * self.error) + (self.kd * (self.error - self.prev_error) / self.del_t) + self.ki * self.integral 

        # adjust speed according to steering angle
        if (angle > (20.0 * np.pi / 180.0)):
            self.speed = 0.5
        elif (angle > (10.0 * np.pi / 180.0)):
            self.speed = 1.0
        else:
            self.speed = 1.5

        # fill in drive message and publish
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = self.speed

        self.ackermann_publisher.publish(drive_msg)

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        # replace with error calculated by get_error()
        self.prev_error = self.error
        self.error = self.get_error(msg, self.desired_distance)
        # actuate the car with PID
        self.pid_control() 

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