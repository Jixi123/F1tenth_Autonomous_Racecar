#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

import numpy as np
import sys
import csv
import signal

class WaypointLogger(Node):
    def __init__(self):
        super().__init__('waypoint_logger_node')

        odom_topic = "/pf/viz/inferred_pose"

        # subscribe to odometry topic
        self.odom_sub = self.create_subscription(PoseStamped, odom_topic, self.odom_callback, 10)
        self.clicked_waypoints = np.empty((0, 2))

        self.x = 0.0
        self.y = 0.0

        # Timer to publish messages as fast as possible
        self.timer = self.create_timer(0.2, self.timer_callback)

    def odom_callback(self, odom_msg):
        self.x = odom_msg.pose.position.x
        self.y = odom_msg.pose.position.y
    
    def timer_callback(self):
        new_waypoint = np.array([[self.x, self.y]])
        self.clicked_waypoints = np.concatenate((self.clicked_waypoints, new_waypoint), axis=0)


def save_recorded_waypoints(waypoints, filename):
    with open(filename, 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        for waypoint in waypoints:
            csv_writer.writerow(waypoint)

        
def main(args=None):
    rclpy.init(args=args)
    print("Record Waypoints Initialized")
    waypoint_logger = WaypointLogger()

    def sigint_handler(sig, frame):
        nonlocal waypoint_logger
        print("Received termination signal. Shutting down...")
        waypoint_logger.destroy_node()
        rclpy.shutdown()

        # record clicked waypoints and save to csv
        recorded_waypoints = waypoint_logger.clicked_waypoints
        save_recorded_waypoints(recorded_waypoints, "/home/team5/f1tenth_ws/src/pure_pursuit/waypoints/race_teleop.csv")

        sys.exit(0)

    signal.signal(signal.SIGINT, sigint_handler)

    rclpy.spin(waypoint_logger)


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    waypoint_logger.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()




