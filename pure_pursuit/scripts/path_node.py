#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Odometry, Path

class WaypointPathNode(Node):
    
    def __init__(self):
        super().__init__('path_node')
        pf_odom_topic = '/pf/pose/odom'
        path_topic = '/path'

        self.odom_sub = self.create_subscription(Odometry, pf_odom_topic, self.pose_callback, 10)
        self.path_pub = self.create_publisher(Path, path_topic, 10)

        self.waypoints = np.genfromtxt('/home/team5/f1tenth_ws/src/pure_pursuit/waypoints/race.csv', delimiter=',')
        self.waypoints = self.waypoints[:, 0 : 2]

        self.path_msg = Path()
        self.path_msg.header.stamp.sec = int(self.get_clock().now().nanoseconds // 1e9)
        self.path_msg.header.frame_id = "map"
        
        for i in range(self.waypoints.shape[0]):
            loc = PoseStamped()
            loc.header.stamp.sec = int(self.get_clock().now().nanoseconds // 1e9)

            loc.header.frame_id = "map"
            loc.pose.position.x = self.waypoints[i, 0]
            loc.pose.position.y = self.waypoints[i, 1]
            loc.pose.position.z = 0.00
            self.path_msg.poses.append(loc)
        
        
    def pose_callback(self, pose_msg):
        self.path_pub.publish(self.path_msg)


def main(args=None):
    rclpy.init(args=args)
    print("Path Node Initialized")
    path_node = WaypointPathNode()
    rclpy.spin(path_node)

    path_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
