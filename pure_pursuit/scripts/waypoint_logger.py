#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PointStamped
import csv
from std_msgs.msg import String 
from nav_msgs.msg import Path 
from geometry_msgs.msg import PoseStamped


class WaypointLogger(Node):
    def __init__(self):
        super().__init__('waypoint_logger')
        self.log_sub = self.create_subscription(PointStamped, '/clicked_point', self.waypoint_callback,10)
        self.csv_file = open('./src/pure_pursuit/waypoints/path.csv', 'w')
        self.csv_writer = csv.writer(self.csv_file)
        self.get_logger().info("logger started")

    def close_csv(self):
        self.csv_file.close
        
    def waypoint_callback(self, msg):
        x = msg.point.x
        y = msg.point.y 
        self.csv_writer.writerow([x,y])
        
def main(args=None):
    rclpy.init(args=args)
    print("waypoint logger initialized")
    waypoint_logger = WaypointLogger()
    rclpy.spin(waypoint_logger)
    waypoint_logger.close_csv()
    waypoint_logger.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
