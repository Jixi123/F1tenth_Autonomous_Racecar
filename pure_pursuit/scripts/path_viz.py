#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PointStamped
import csv
from std_msgs.msg import String 
from nav_msgs.msg import Path 
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry 

class PathDisplay(Node):
    def __init__(self):
        super().__init__('path_viz')
        
        self.path = Path()
        self.path_pub = self.create_publisher(Path, '/path', 10)
        self.odo_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.odo_callback, 10)
        self.waypoints = np.loadtxt("./src/pure_pursuit/waypoints/path.csv", delimiter = ",")
        self.get_logger().info("path initialized")
        print(self.waypoints[0][0])
        print(self.waypoints[0][1])
        for i in range(len(self.waypoints)):
            point = PoseStamped()
            point.pose.position.x = self.waypoints[i][0]
            point.pose.position.y = self.waypoints[i][1]
            point.pose.position.z = 0.0
            self.path.poses.append(point)
    
    def odo_callback(self, msg):
        self.path_pub.publish(self.path)
        
def main(args=None):
    rclpy.init(args=args)
    path_display= PathDisplay()
    rclpy.spin(path_display)
    path_display.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
