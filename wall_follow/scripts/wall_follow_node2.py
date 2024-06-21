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
        
        self.get_logger().info("wall following started")

        self.scan_sub = self.create_subscription(LaserScan, lidarscan_topic, self.scan_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

        self.kp = 0.5
        self.kd = 0.6
        self.ki = 0.01

        self.integral = 0.0
        self.prev_error = 0.0
        self.error = 0.0

        self.desired_distance = 1
        self.look_distance = 1.2
        self.a_angle = 180
        self.b_angle = 225
    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """
        ranges = range_data.ranges
        increment_val = range_data.angle_increment
        index = int(np.radians(angle) // increment_val)
        while(ranges[index] == float('inf') or np.isnan(ranges[index])):
            index += 1
        dist = ranges[index]
        return dist

    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """
        theta = self.b_angle - self.a_angle
        a = self.get_range(range_data, self.a_angle)
        b = self.get_range(range_data, self.b_angle)
        alpha = np.arctan((a*np.cos(theta)-b)/(a*np.sin(theta)))
        D_t = b*np.cos(alpha)
        D_t1 = D_t + self.look_distance * np.sin(alpha)
        error = self.desired_distance - D_t1
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
        self.integral += error * 0.04
        angle = self.kp*error  + self.ki * self.integral + self.kd * (error-self.prev_error)/0.04
        self.prev_error = error
        # TODO: Use kp, ki & kd to implement a PID controller
        drive_msg = AckermannDriveStamped()
        # TODO: fill in drive message and publish
        drive_msg.drive.steering_angle = -angle
        angle = np.abs(np.degrees(angle))
        if(0<=angle<10):
            drive_msg.drive.speed = 1.5
        elif(10<=angle<20):
            drive_msg.drive.speed = 1.0
        else:
            drive_msg.drive.speed = 0.5
            
        #delete later
        drive_msg.drive.speed = 1.0
        drive_msg.drive.steering_angle = 0.0
        
        self.drive_pub.publish(drive_msg)
        self.get_logger().info("driving at " + str(drive_msg.drive.speed))
        
    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        self.error = self.get_error(msg, self.desired_distance)
        0 # TODO: replace with error calculated by get_error()
        velocity = 0.0 # TODO: calculate desired car velocity based on error
        self.pid_control(self.error, velocity) # TODO: actuate the car with PID


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