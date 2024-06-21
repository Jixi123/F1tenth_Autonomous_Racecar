#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Odometry, Path

class VizLocalizationNode(Node):
    
    def __init__(self):
        super().__init__('viz_node')
        odom_topic = '/odometry/filtered'
        viz_topic = '/ukf_pose'

        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.pose_callback, 10)
        self.viz_pub = self.create_publisher(PoseStamped, viz_topic, 10)
        
        
    def pose_callback(self, pose_msg):
        x = pose_msg.pose.pose.position.x
        y = pose_msg.pose.pose.position.y
        z = pose_msg.pose.pose.position.z
        x_quat = pose_msg.pose.pose.orientation.x
        y_quat = pose_msg.pose.pose.orientation.y
        z_quat = pose_msg.pose.pose.orientation.z
        w_quat = pose_msg.pose.pose.orientation.w

        loc = PoseStamped()
        loc.header.stamp.sec = int(self.get_clock().now().nanoseconds // 1e9)
        loc.header.frame_id = "map"
        loc.pose.position.x = x
        loc.pose.position.y = y
        loc.pose.position.z = z
        loc.pose.orientation.x = x_quat
        loc.pose.orientation.y = y_quat
        loc.pose.orientation.z = z_quat
        loc.pose.orientation.w = w_quat

        self.viz_pub.publish(loc)


def main(args=None):
    rclpy.init(args=args)
    print("Vizualization Initialized")
    viz_node = VizLocalizationNode()
    rclpy.spin(viz_node)

    viz_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


