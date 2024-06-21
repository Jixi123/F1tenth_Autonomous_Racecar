#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

import numpy as np
import sys
import signal
import csv
from scipy.interpolate import splprep, splev
import matplotlib.pyplot as plt

class RecordWaypointsNode(Node):
    def __init__(self):
        super().__init__('record_waypoints_node')

        # subscribe to clicked_points topic
        self.clicked_pts_sub = self.create_subscription(PointStamped, '/clicked_point', self.callback, 10)

        # clicked waypoints
        self.clicked_waypoints = np.empty((0, 2))

    def callback(self, msg):
        point_x = msg.point.x
        point_y = msg.point.y
        print("point: ", point_x, point_y)
        new_waypoint = np.array([[point_x, point_y]])
        self.clicked_waypoints = np.concatenate((self.clicked_waypoints, new_waypoint), axis=0)


def save_recorded_waypoints(waypoints, filename):
    with open(filename, 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        for waypoint in waypoints:
            csv_writer.writerow(waypoint)

def interpolate_waypoints(waypoints):
    waypoints_x = waypoints[:, 0]
    waypoints_y = waypoints[:, 1]
    tck, u = splprep([waypoints_x, waypoints_y], s=0)
    new_x, new_y = splev(u, tck)

    new_points = np.vstack((new_x, new_y)).T

    fig, ax = plt.subplots()
    ax.plot(waypoints_x, waypoints_y, 'ro')
    ax.plot(new_x, new_y, 'r-')
    plt.savefig('waypoints_tepper.png')

    return new_points

def main(args=None):
    rclpy.init(args=args)
    print("Record Waypoints Initialized")
    record_waypoints_node = RecordWaypointsNode()

    def sigint_handler(sig, frame):
        nonlocal record_waypoints_node
        print("Received termination signal. Shutting down...")
        record_waypoints_node.destroy_node()
        rclpy.shutdown()

        # record clicked waypoints and save to csv
        recorded_waypoints = record_waypoints_node.clicked_waypoints
        save_recorded_waypoints(recorded_waypoints, "waypoints_tepper.csv")

        # interpolate between waypoints and save to csv
        interpolated_waypoints = interpolate_waypoints(recorded_waypoints)
        save_recorded_waypoints(interpolated_waypoints, "interpolated_tepper.csv")

        sys.exit(0)

    signal.signal(signal.SIGINT, sigint_handler)

    rclpy.spin(record_waypoints_node)


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    record_waypoints_node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()