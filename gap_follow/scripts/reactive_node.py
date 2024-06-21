#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class ReactiveFollowGap(Node):
    """ 
    Implement Wall Following on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('reactive_node')
        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: Subscribe to LIDAR
        # TODO: Publish to drive
        self.scan_subscriber = self.create_subscription(LaserScan, lidarscan_topic, self.lidar_callback, 10)
        self.ackermann_publisher = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

        self.sliding_window_length = 5

        self.car_width = 0.5
        self.disparity_tolerance = 1.0
        self.max_lidar_distance = 3.0
        self.best_conv_size = 80


    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        proc_ranges = np.zeros(len(ranges))

        temp = np.zeros(len(ranges))
        running_avg = []

        for i in range(len(ranges)):
            running_avg.append(ranges[i])
            if len(running_avg) > self.sliding_window_length:
                running_avg.pop(0)

            running_avg_sum = sum(running_avg)
            temp[i] = running_avg_sum / len(running_avg)

        for i in range(len(ranges)):
            proc_ranges[i] = temp[i]

        proc_ranges = np.clip(proc_ranges, 0, self.max_lidar_distance)

        return proc_ranges
    

    def find_best_point(self, ranges):
        """
        Combination of find max gap and find best point
        Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	    Naive: Choose the furthest point within ranges and go there
        """
        max_start = 0
        max_end = 0
        start = 0
        max_length = 0
        length = 0

        for i in range(len(ranges)):
            if ranges[i] > 0:
                length += 1
                if length > max_length:
                    max_length = length
                    max_start = start
                    max_end = i
            else:
                length = 0
                start = i + 1

        averaged_max_gap = np.convolve(ranges[max_start:max_end], np.ones(self.best_conv_size),
                                       'same') / self.best_conv_size
        return averaged_max_gap.argmax() + max_start
    

    def extend_disparity(self, ranges, angle_increment):
        i = 0
        proc_ranges = np.copy(ranges)
        while (i < len(ranges) - 1):
            if (np.abs(ranges[i] - ranges[i + 1]) > self.disparity_tolerance):
                # extend by half car width
                angle = np.arctan(self.car_width / (min(ranges[i], ranges[i + 1])))
                num_scans_to_modify = int(angle / angle_increment)
                if ranges[i] < ranges[i + 1]:
                    # extend disparity forward
                    for j in range(i + 1, min(len(ranges), i + 1 + num_scans_to_modify)):
                        proc_ranges[j] = ranges[i]
                    i = i + num_scans_to_modify + 1
                else:
                    # extend disparity backwards
                    for j in range(i - 1, max(0, i - 1 - num_scans_to_modify), -1):
                        proc_ranges[j] = ranges[i + 1]
                    i = i + 1
            else:
                i = i + 1
        return proc_ranges

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges

        min_angle = -70.0 * np.pi / 180.0
        max_angle = 70.0 * np.pi / 180.0
        min_indx = int((min_angle - data.angle_min) / data.angle_increment)
        max_indx = int((max_angle - data.angle_min) / data.angle_increment)

        for i in range(min_indx, max_indx + 1):
            if np.isinf(ranges[i]) or np.isnan(ranges[i]):
                ranges[i] = data.range_max
            elif ranges[i] < data.range_min:
                ranges[i] = data.range_min
            elif ranges[i] > data.range_max:
                ranges[i] = data.range_max

        proc_ranges = self.preprocess_lidar(ranges)
        proc_ranges = self.extend_disparity(proc_ranges, data.angle_increment)

        # TODO:
        #Find closest point to LiDAR
        closest_indx = min_indx
        closest_distance = data.range_max 
        for i in range(min_indx, max_indx + 1):
            distance = proc_ranges[i]
            if distance < closest_distance:
                closest_distance = distance
                closest_indx = i

        #Eliminate all points inside 'bubble' (set them to zero) 
        radius = 170    # 170
        for i in range(max(0, closest_indx - radius), min(len(proc_ranges), closest_indx + radius + 1)):
            proc_ranges[i] = 0.0

        target_indx = self.find_best_point(proc_ranges)

        steering_angle = data.angle_min + target_indx * data.angle_increment

        #Publish Drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steering_angle

        # adjust speed according to steering angle
        if (np.abs(steering_angle) > (20.0 * np.pi / 180.0)):
            drive_msg.drive.speed = 1.0
        elif (np.abs(steering_angle) > (10.0 * np.pi / 180.0)):
            drive_msg.drive.speed = 1.5
        else:
            drive_msg.drive.speed = 2.0

        self.ackermann_publisher.publish(drive_msg)
        self.get_logger().info("steering: " + str(np.degrees(drive_msg.drive.steering_angle)))

def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

