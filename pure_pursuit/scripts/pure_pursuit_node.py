#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped

import numpy as np

# TODO CHECK: include needed ROS msg type headers and libraries

class PurePursuit(Node):
    """ 
    Implement Pure Pursuit on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('pure_pursuit_node')
        # TODO: create ROS subscribers and publishers
        pf_odom_topic = '/pf/viz/inferred_pose' # change to particle filter for actual car
        drive_topic = '/drive'

        self.odom_sub = self.create_subscription(PoseStamped, pf_odom_topic, self.pose_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        self.path_pub = self.create_publisher(Marker,'/visualization_marker', 10)
        self.waypoints = np.genfromtxt('/home/team5/f1tenth_ws/src/pure_pursuit/waypoints/race.csv', delimiter=',')
        self.waypoints = self.waypoints[:, 0 : 2]

        self.lookahead = 1.0
        self.curr_index = 0
        self.clamp_angle = 25.0
        self.heading_current = 0.0
        self.base_to_lidar = 0.27275    # m

        self.x_current = 0.0
        self.y_current = 0.0


    def display_marker(self, current_waypoint):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.ns = "marker"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = 0
        marker.type = 2
        marker.action = 0
        marker.pose.position.x = current_waypoint[0]
        marker.pose.position.y = current_waypoint[1]
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.25
        marker.scale.z = 0.1
        marker.scale.y = 0.25
        marker.color.a = 1.0
        marker.color.g = 1.0
        self.path_pub.publish(marker)


    def convert_map_to_base(self, x_map, y_map):
        """
        Converts map coordinates to base link frame coordinates
        """
        x_base_rot = x_map - self.x_current
        y_base_rot = y_map - self.y_current
        angle = -self.heading_current
        rot_matrix = [[np.cos(angle), -np.sin(angle)],
                      [np.sin(angle), np.cos(angle)]]
        [x_grid, y_grid] = np.matmul(rot_matrix, [x_base_rot, y_base_rot])

        return (x_grid, y_grid)
   
    def get_current_waypoint(self, location):
        num_points = self.waypoints.shape[0]
        closest_distance = 1000.0
    
        for i in range(num_points):
            x_waypoint_grid, y_waypoint_grid = self.convert_map_to_base(self.waypoints[i, 0], self.waypoints[i, 1])
            if (x_waypoint_grid < self.base_to_lidar):
                continue
            distance = np.sqrt((self.waypoints[i, 0] - location[0])**2 + (self.waypoints[i, 1] - location[1])**2)
            # dist = np.abs(distance - self.lookahead)
            dist = distance
            if dist < closest_distance:
                self.curr_index = i
                closest_distance = dist

        waypoint_prev = self.waypoints[self.curr_index]
        waypoint_next = self.waypoints[min(self.curr_index + 1, num_points - 1)]

        current_waypoint = (waypoint_prev + waypoint_next) / 2
        self.display_marker(current_waypoint)

        return current_waypoint


    def get_speed(self, angle):
        abs_angle = np.abs(angle)
        if abs_angle >= np.deg2rad(20):
            speed = 1.0
        elif abs_angle >= np.deg2rad(15):
            speed = 1.25
        elif abs_angle >= np.deg2rad(10):
            speed = 2.0
        else:
            speed = 2.5
        return speed

    def pose_callback(self, pose_msg):
        x = pose_msg.pose.orientation.x
        y = pose_msg.pose.orientation.y
        z = pose_msg.pose.orientation.z
        w = pose_msg.pose.orientation.w

        self.x_current = pose_msg.pose.position.x
        self.y_current = pose_msg.pose.position.y

        q = [w, x, y, z]
        sin_angle = 2.0 * (q[0] * q[3] + q[1] * q[2])
        cos_angle = 1.0 - 2.0 * (q[2]**2 + q[3]**2)
        self.heading_current = np.arctan2(sin_angle, cos_angle)

        rot_matrix = np.array([[1-2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w], 
                                [2*x*y + 2*z*w, 1-2*x**2 - 2*z**2, 2*y*z - 2*x*w],
                                [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1-2*x**2 - 2*y**2]])
        
        location = [self.x_current, self.y_current]

        # find the current waypoint to track using methods mentioned in lecture
        target_waypoint = self.get_current_waypoint(location)

        angle = 0.0

        # transform goal point to vehicle frame of reference
        goal_point_wrt_world = target_waypoint - location
        goal_point_wrt_world = np.append(goal_point_wrt_world, 0.0)
        goal_point_wrt_body = np.matmul(np.linalg.inv(rot_matrix), goal_point_wrt_world)

        # calculate curvature/steering angle
        angle = (2 * goal_point_wrt_body[1]) / (self.lookahead ** 2)
        angle = np.clip(angle, -np.deg2rad(self.clamp_angle), np.deg2rad(self.clamp_angle))

        # TODO: publish drive message, don't forget to limit the steering angle.
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = self.get_speed(angle)

        self.drive_pub.publish(drive_msg)
        print("steering at angle: ", np.rad2deg(angle))


def main(args=None):
    rclpy.init(args=args)
    print("PurePursuit Initialized")
    pure_pursuit_node = PurePursuit()
    rclpy.spin(pure_pursuit_node)

    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
