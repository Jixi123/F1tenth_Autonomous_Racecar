#!/usr/bin/env python3
"""
This file contains the class definition for tree nodes and RRT
Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
"""
import numpy as np
from numpy import linalg as LA
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker

# TODO: import as you need
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import scipy

import time

# class def for tree nodes
# It's up to you if you want to use this
class TreeNode(object):
    def __init__(self):
        self.x = None
        self.y = None
        self.parent = None
        self.cost = None # only used in RRT*
        self.is_root = False

# class def for RRT
class RRT(Node):
    def __init__(self):
        super().__init__('rrt')
        # topics, not saved as attributes
        # TODO: grab topics from param file, you'll need to change the yaml file
        pose_topic = "/pf/viz/inferred_pose"     # TBD: will need to be updated
        scan_topic = "/scan"
        drive_topic = "/drive"
        og_topic = "/dynamic_map"

        # you could add your own parameters to the rrt_params.yaml file,
        # and get them here as class attributes as shown above.

        # TODO: create subscribers
        self.pose_sub_ = self.create_subscription(      # TBD: will need to be updated
            #PoseStamped,
            PoseStamped,
            pose_topic,
            self.pose_callback,
            1)

        self.scan_sub_ = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            1)

        # publishers
        # TODO: create a drive message publisher, and other publishers that you might need
        self.drive_pub_ = self.create_publisher(AckermannDriveStamped, drive_topic, 1)
        self.og_pub_ = self.create_publisher(OccupancyGrid, og_topic, 1)

        # debugging
        self.tree_nodes_pub_ = self.create_publisher(Marker, '/tree_nodes', 10)
        self.path_pub_ = self.create_publisher(Path, '/found_path', 10)
        self.marker1_pub_ = self.create_publisher(Marker,'/goal_waypoint', 1)
        self.marker2_pub_ = self.create_publisher(Marker,'/local_target', 1)
        self.marker3_pub_ = self.create_publisher(Marker,'/samples', 1)

        # class attributes

        # occupancy grid attributes
        self.og_height = 2.0            # m
        self.og_width = 3.0             # m
        self.og_resolution = 0.05       # m
        self.kernel_size = 9

        # odometry attributes (global - map frame)
        self.x_current = 0.0
        self.y_current = 0.0
        self.heading_current = 0.0

        # global planner parameters
        self.x_current_goal = 0.0       
        self.y_current_goal = 0.0  
        self.global_waypoints = np.genfromtxt('/home/team5/f1tenth_ws/src/HMPC/waypoints/practice1_waypoints.csv', delimiter=',')
        self.global_waypoints = self.global_waypoints[:, 0 : 2]     

        # physical car attributes
        self.base_to_lidar = 0.27275    # m
        self.edge_to_lidar = 0.10743    # m

        # RRT parameters
        self.max_rrt_iterations = 1000
        self.lookahead_distance = 2.0   # m
        self.steer_range = 0.3          # m``
        self.goal_tolerance = 0.25       # m
        self.collision_checking_points = 20

        # RRT* parameters
        self.enable_rrt_star = False
        self.search_radius = 2.0        # m

        # sampling parameters
        self.sample_bias = 0.3
        self.std_deviation = 0.5

        # pure pursuit parameters
        self.clamp_angle = 25.0         # deg
        self.steering_gain = 1.0

        # initialize occupancy grid
        self.occupancy_grid = OccupancyGrid()
        self.init_occupancy_grid()


    def init_occupancy_grid(self):
        """
        Initialize occupancy grid 
        """
        rows = int(self.og_height // self.og_resolution)
        cols = int(self.og_width // self.og_resolution)
        self.occupancy_grid.header.frame_id = "laser"
        self.occupancy_grid.info.width = cols
        self.occupancy_grid.info.height = rows
        self.occupancy_grid.info.resolution = self.og_resolution
        self.occupancy_grid.header.stamp = self.get_clock().now().to_msg()
        data = np.full((rows, cols), -1, np.int8)
        self.occupancy_grid.data = data.flatten().tolist()
        # self.occupancy_grid.info.origin.position.x = -self.base_to_lidar
        self.occupancy_grid.info.origin.position.x = 0.0
        self.occupancy_grid.info.origin.position.y = -(rows // 2) * self.og_resolution
        self.occupancy_grid.info.origin.position.z = 0.0
        
        self.og_pub_.publish(self.occupancy_grid)

    
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


    def convert_base_to_map(self, x_base, y_base):
        """
        Converts base link frame coordinates to map frame
        """
        # rotate to parallel to map frame
        rot_matrix = [[np.cos(self.heading_current), -np.sin(self.heading_current)],
                        [np.sin(self.heading_current), np.cos(self.heading_current)]]
        [x_base_rot, y_base_rot] = np.matmul(rot_matrix, [x_base, y_base])

        # translate by odom
        x_map = x_base_rot + self.x_current
        y_map = y_base_rot + self.y_current

        return (x_map, y_map)
    

    def is_occupied(self, x_grid, y_grid):
        """
        Checks if lidar coordinate x, y is occupied in occupancy grid
        """
        # get corresponding cell in occupany grid
        row = int(self.occupancy_grid.info.height // 2 + y_grid // self.og_resolution)
        col = int(x_grid // self.og_resolution)

        if (row < 0 or col < 0):
            return False

        if (row >= self.occupancy_grid.info.height or col >= self.occupancy_grid.info.width):
            return False
        
        og_index = int(row * self.occupancy_grid.info.width + col)

        if (self.occupancy_grid.data[og_index] > 0):
            return True
        else:
            return False
    

    def get_next_goal(self):
        """
        Gets next global planner waypoint to get to using RRT
        """
        best_index = -1
        best_goal_distance = 10000.0
        for i in range(len(self.global_waypoints)):
            global_x = self.global_waypoints[i][0]
            global_y = self.global_waypoints[i][1]
            (global_x_grid, global_y_grid) = self.convert_map_to_base(global_x, global_y)

            if (global_x_grid < self.base_to_lidar):
                # goal behind car, skip
                continue

            goal_dist = np.abs(self.lookahead_distance - np.sqrt(global_x_grid**2 + global_y_grid**2))

            if (goal_dist < best_goal_distance):
                # make sure it is not an occupied point
                # if (self.is_occupied(global_x_grid, global_y_grid)):
                #     continue

                best_goal_distance = goal_dist
                best_index = i

        return (self.global_waypoints[best_index][0], self.global_waypoints[best_index][1])
        

    def display_marker(self, frame, r, g, b, current_waypoint):
        marker = Marker()
        marker.header.frame_id = frame
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
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        return marker
    

    def visualize_tree(self, tree):
        tree_nodes = Marker()
        tree_nodes.header.frame_id = "map"
        tree_nodes.ns = "marker"
        tree_nodes.header.stamp = self.get_clock().now().to_msg()
        tree_nodes.id = 1
        tree_nodes.type = 8
        tree_nodes.action = 0
        tree_nodes.scale.x = 0.1
        tree_nodes.scale.z = 0.1
        tree_nodes.scale.y = 0.1
        tree_nodes.color.a = 1.0
        tree_nodes.color.r = 0.0
        tree_nodes.color.g = 0.0
        tree_nodes.color.b = 1.0

        for node in tree:
            point = Point()
            point.x = node.x
            point.y = node.y
            point.z = 0.0
            tree_nodes.points.append(point)

        self.tree_nodes_pub_.publish(tree_nodes)
        tree_nodes.points.clear()

    
    def visualize_path(self, path):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        
        for node in path:
            loc = PoseStamped()
            loc.header.stamp = self.get_clock().now().to_msg()

            loc.header.frame_id = "map"
            loc.pose.position.x = node.x
            loc.pose.position.y = node.y
            loc.pose.position.z = 0.00
            path_msg.poses.append(loc)

        self.path_pub_.publish(path_msg)


    def preprocess_lidar(self, ranges):
        pass


    def get_speed(self, angle):
        abs_angle = np.abs(angle)
        if abs_angle >= np.deg2rad(15):
            speed = 0.75
        elif abs_angle >= np.deg2rad(10):
            speed = 1.0
        elif abs_angle >= np.deg2rad(5):
            speed = 1.5
        else:
            speed = 2.0
        # speed = 0.5
        return speed
        #return 0.0
    

    def inflate_obstacles(self, kernel):

        height = self.occupancy_grid.info.height
        width = self.occupancy_grid.info.width

        # Get kernel dimensions
        k_height, k_width = kernel.shape

        # Compute padding for the input image
        pad_height = k_height // 2
        pad_width = k_width // 2

        og_grid_data = np.array(self.occupancy_grid.data)
        og_grid_data = og_grid_data.reshape((height, width))

        # Create an empty output image
        dilated_image = np.zeros_like(og_grid_data)

        # Pad the input image
        padded_image = np.pad(og_grid_data, ((pad_height, pad_height), (pad_width, pad_width)), mode='constant')

        # # Apply dilation
        for y in range(height):
            for x in range(width):
                neighborhood = padded_image[y:y + k_height, x:x + k_width]
                dilated_image[y, x] = np.max(neighborhood * kernel)
        
        self.occupancy_grid.data = dilated_image.flatten().tolist()  


    def scan_callback(self, scan_msg):
        """
        LaserScan callback, you should update your occupancy grid here

        Args: 
            scan_msg (LaserScan): incoming message from subscribed topic
        Returns:
        """
        ranges = np.array(scan_msg.ranges)
        rows = self.occupancy_grid.info.height
        cols = self.occupancy_grid.info.width

        proc_ranges = np.copy(ranges)
        proc_ranges[proc_ranges < scan_msg.range_min] = scan_msg.range_min
        proc_ranges[proc_ranges > scan_msg.range_max] = scan_msg.range_max
        proc_ranges[np.isnan(proc_ranges) | np.isinf(proc_ranges)] = scan_msg.range_max

        # self.preprocess_lidar(proc_ranges)

        # Create meshgrid of row and col indices
        col_indices, row_indices = np.meshgrid(np.arange(cols), np.arange(rows))

        # Calculate x and y coordinates for each cell
        x_cell = col_indices * self.og_resolution + (self.og_resolution / 2)
        y_cell = (row_indices - (rows / 2)) * self.og_resolution + (self.og_resolution / 2)

        # Calculate distance to each cell
        distance_to_cell = np.sqrt(x_cell**2 + y_cell**2)

        # Calculate angle to each cell
        angle_to_cell = np.arctan2(y_cell, x_cell)

        # Find closest index in scan_msg for each cell
        closest_index = ((angle_to_cell - scan_msg.angle_min) / scan_msg.angle_increment).astype(int)

        # Get distance to object for each cell
        distance_to_obj = proc_ranges[closest_index]

        # Create occupancy grid data
        occupancy_grid_data = np.where(distance_to_cell >= distance_to_obj, 100, 0)

        # Flatten the occupancy grid data and assign it to self.occupancy_grid.data
        self.occupancy_grid.data = occupancy_grid_data.flatten().tolist()

        kernel = np.ones((self.kernel_size, self.kernel_size))
        self.inflate_obstacles(kernel)
        self.occupancy_grid.header.stamp = self.get_clock().now().to_msg()

        self.og_pub_.publish(self.occupancy_grid)


    def pose_callback(self, pose_msg):
        """
        The pose callback when subscribed to particle filter's inferred pose
        Here is where the main RRT loop happens

        Args: 
            pose_msg (PoseStamped): incoming message from subscribed topic
        Returns:

        """
        # TBD: will need to be updated for running on car
        self.x_current = pose_msg.pose.position.x
        self.y_current = pose_msg.pose.position.y

        # convert rotation quaternion to heading angle
        q = [pose_msg.pose.orientation.w, pose_msg.pose.orientation.x,
            pose_msg.pose.orientation.y, pose_msg.pose.orientation.z]
        sin_angle = 2.0 * (q[0] * q[3] + q[1] * q[2])
        cos_angle = 1.0 - 2.0 * (q[2]**2 + q[3]**2)
        self.heading_current = np.arctan2(sin_angle, cos_angle)

        # update next goal global waypoint
        (self.x_current_goal, self.y_current_goal) = self.get_next_goal()
        (goal_x_grid, goal_y_grid) = self.convert_map_to_base(self.x_current_goal, self.y_current_goal)
        dist_to_goal = (self.x_current - self.x_current_goal)**2 + (self.y_current - self.y_current_goal)**2

        if (self.is_occupied(goal_x_grid, goal_y_grid)):
            print("goal occupied")
            self.goal_tolerance = 0.9
        else:
            self.goal_tolerance = 0.25

        # debugging
        p_map = self.display_marker("map", 1.0, 0.0, 0.0, [self.x_current_goal, self.y_current_goal])
        self.marker1_pub_.publish(p_map)

        # define starter node
        start_node = TreeNode()
        start_node.x = self.x_current
        start_node.y =  self.y_current
        start_node.is_root = True
        start_node.parent = 0
        start_node.cost = 0.0

        # initialize tree
        tree = [start_node]

        for iter in range(self.max_rrt_iterations):
            sampled_point = self.sample()
            dist_to_sample = (self.x_current - sampled_point[0])**2 + (self.y_current - sampled_point[1])**2

            (sample_x_grid, sample_y_grid) = self.convert_map_to_base(sampled_point[0], sampled_point[1])
            if ((self.is_occupied(sample_x_grid, sample_y_grid)) or (dist_to_sample > dist_to_goal)):
                continue
            # debugging
            # grid_point = self.convert_map_to_base(sampled_point[0], sampled_point[1])
            # p_grid = self.display_marker("ego_racecar/laser", 0.0, 1.0, 0.0, [grid_point[0], grid_point[1]])
            # self.marker2_pub_.publish(p_grid)

            nearest_indx = self.nearest(tree, sampled_point)

            new_node = self.steer(tree[nearest_indx], sampled_point)
            
            (new_x_grid, new_y_grid) = self.convert_map_to_base(new_node.x, new_node.y)
            if (self.is_occupied(new_x_grid, new_y_grid)):
                continue

            new_node.parent = nearest_indx
            current_node_index = len(tree)

            if (not self.check_collision(tree[nearest_indx], new_node)):
                
                # run RRT* if enabled
                if (self.enable_rrt_star):
                    new_node.cost = self.cost(tree, new_node)
                    near_neighbour_indices = self.near(tree, new_node)
                    # initialize array of bools of neighbours which collide
                    neighbour_collisions = []                  
                    best_neighbour = new_node.parent

                    for near_node_index in near_neighbour_indices:
                        # check which neighbours collide
                        if (self.check_collision(tree[near_node_index], new_node)):
                            neighbour_collisions.append(True)
                            continue

                        neighbour_collisions.append(False)
                        # update best neighbour based on cost
                        cost_value = tree[near_node_index].cost + self.line_cost(tree[near_node_index], new_node)
                        if (cost_value < new_node.cost):
                            new_node.cost = cost_value
                            new_node.parent = near_node_index
                            best_neighbour = near_node_index

                    for i in range(len(near_neighbour_indices)):
                        if (i == best_neighbour) or (neighbour_collisions[i]):
                            continue

                        near_neighbour = tree[near_neighbour_indices[i]]
                        if (near_neighbour.cost > new_node.cost + self.line_cost(new_node, near_neighbour)):
                            # rewire 
                            near_neighbour.parent = current_node_index

                if ((self.is_occupied(new_x_grid, new_y_grid))):
                    p_map = self.display_marker("map", 0.0, 1.0, 0.0, [new_node.x, new_node.y])
                    self.marker3_pub_.publish(p_map)

                tree.append(new_node)

                if (self.is_goal(new_node, self.x_current_goal, self.y_current_goal)):
                    # close enough to goal
                    path = self.find_path(tree, new_node)
                    self.track_path(path)
                    # self.visualize_tree(tree)
                    self.visualize_path(path)
                    break


    def sample(self):
        """
        This method should randomly sample the free space, and returns a viable point

        Args:
        Returns:
            (x, y) (float float): a tuple representing the sampled point

        """
        # x = np.random.uniform(0.0, self.og_width) # lidar frame
        # y = np.random.uniform(- self.og_height / 2, self.og_height / 2)   # lidar frame

        (goal_grid_x, goal_grid_y) = self.convert_map_to_base(self.x_current_goal, self.y_current_goal)

        if np.random.rand() < self.sample_bias:
            # Sample near the goal point using Gaussian distribution
            x = np.random.normal(goal_grid_x, self.std_deviation)
            y = np.random.normal(goal_grid_y, self.std_deviation)
        else:
            # Sample uniformly in the free space
            x = np.random.uniform(0.0, self.og_width)  # lidar frame
            y = np.random.uniform(-self.og_height / 2, self.og_height / 2)  # lidar frame
        
        # if ((not self.is_occupied(x, y)) and (x <= goal_grid_x) and (x >= self.base_to_lidar)):
        #     # convert to map coordinates from grid coordinates

        (x_map, y_map) = self.convert_base_to_map(x, y)

        return (x_map, y_map)
        # else:
        #     return self.sample()


    def nearest(self, tree, sampled_point):
        """
        This method should return the nearest node on the tree to the sampled point

        Args:
            tree ([]): the current RRT tree
            sampled_point (tuple of (float, float)): point sampled in free space
        Returns:
            nearest_node (int): index of neareset node on the tree
        """
        nearest_node = 0
        # min_dist = 10000.0
        # for i in range(len(tree)):
        #     node = tree[i]
        #     sq_dist = (sampled_point[0] - node.x)**2 + (sampled_point[1] - node.y)**2
        #     if (sq_dist < min_dist):
        #         nearest_node = i
        #         min_dist = sq_dist
        sampled_point = np.array(sampled_point)
        tree_points = np.array([(node.x, node.y) for node in tree])

        # Calculate squared distances between sampled point and all tree nodes
        sq_distances = np.sum((tree_points - sampled_point)**2, axis=1)

        # Find the index of the node with the minimum squared distance
        nearest_node = np.argmin(sq_distances)

        return nearest_node


    def steer(self, nearest_node, sampled_point):
        """
        This method should return a point in the viable set such that it is closer 
        to the nearest_node than sampled_point is.

        Args:
            nearest_node (Node): nearest node on the tree to the sampled point
            sampled_point (tuple of (float, float)): sampled point
        Returns:
            new_node (Node): new node created from steering
        """
        new_node = TreeNode()
        dist = np.sqrt((sampled_point[0] - nearest_node.x)**2 + (sampled_point[1] - nearest_node.y)**2)
        x = sampled_point[0] - nearest_node.x
        y = sampled_point[1] - nearest_node.y

        if (dist < self.steer_range):
            new_node.x = sampled_point[0]
            new_node.y = sampled_point[1]
        else:
            theta = np.arctan2(y, x)
            new_node.x = nearest_node.x + np.cos(theta) * self.steer_range
            new_node.y = nearest_node.y + np.sin(theta) * self.steer_range
    
        # new_node.x = nearest_node.x + min(self.steer_range, dist) * (sampled_point[0] - nearest_node.x) / dist
        # new_node.y = nearest_node.y + min(self.steer_range, dist) * (sampled_point[1] - nearest_node.y) / dist
        return new_node
    

    def check_collision(self, nearest_node, new_node):
        """
        This method should return whether the path between nearest and new_node is
        collision free.

        Args:
            nearest (Node): nearest node on the tree
            new_node (Node): new node from steering
        Returns:
            collision (bool): whether the path between the two nodes are in collision
                              with the occupancy grid
        """
        collision = False
        x_cell_diff = abs(int((nearest_node.x - new_node.x) / self.collision_checking_points))
        y_cell_diff = abs(int((nearest_node.y - new_node.y) / self.collision_checking_points))

        # dt = 1.0 / max(x_cell_diff, y_cell_diff)
        # t = 0.0
        current_x = nearest_node.x
        current_y = nearest_node.y

        for i in range(self.collision_checking_points):
            # x = nearest_node.x + t * (new_node.x - nearest_node.x)
            # y = nearest_node.y + t * (new_node.y - nearest_node.y)
            current_x += x_cell_diff
            current_y += y_cell_diff

            # convert map to grid coordinates to check if occupied
            # (x_grid, y_grid) = self.convert_map_to_base(x, y)
            (x_grid, y_grid) = self.convert_map_to_base(current_x, current_y)

            if (self.is_occupied(x_grid, y_grid)):
                collision = True
                break

            # t += dt

        return collision


    def is_goal(self, latest_added_node, goal_x, goal_y):
        """
        This method should return whether the latest added node is close enough
        to the goal.

        Args:
            latest_added_node (Node): latest added node on the tree
            goal_x (double): x coordinate of the current goal
            goal_y (double): y coordinate of the current goal
        Returns:
            close_enough (bool): true if node is close enoughg to the goal
        """
        distance = np.sqrt((latest_added_node.x - goal_x)**2 + (latest_added_node.y - goal_y)**2)
        return distance < self.goal_tolerance


    def find_path(self, tree, latest_added_node):
        """
        This method returns a path as a list of Nodes connecting the starting point to
        the goal once the latest added node is close enough to the goal

        Args:
            tree ([]): current tree as a list of Nodes
            latest_added_node (Node): latest added node in the tree
        Returns:
            path ([]): valid path as a list of Nodes
        """
        path = []
        current = latest_added_node
        while (not current.is_root):
            path.append(current)
            current = tree[current.parent]
        path.append(current)
        path.reverse()

        (goal_x_grid, goal_y_grid) = self.convert_map_to_base(self.x_current_goal, self.y_current_goal)
        if (not self.is_occupied(goal_x_grid, goal_y_grid)):
            goal_node = TreeNode()
            goal_node.x = self.x_current_goal
            goal_node.y = self.y_current_goal
            path.append(goal_node)
        
        return path
    

    def track_path(self, path):
        """
        Finds node in path just within lookahead distance and follows pure pursuit
        """
        best_index = 0
        closest_distance = 10000.0
        closest_distance_current_pose = 10000.0
        for i in range(len(path)):
            x = path[i].x
            y = path[i].y
            dist = (self.x_current - x) ** 2 + (self.y_current - y) ** 2
            diff_distance = np.abs(self.lookahead_distance - dist)
            # if (dist < self.lookahead_distance ** 2):
            #     best_index = i
            if (diff_distance < closest_distance):
                closest_distance = diff_distance
                best_index = i
                closest_distance_current_pose = dist
        
        # get next point for pure pursuit using average
        p1 = np.array([path[best_index].x, path[best_index].y])
        # p2 = np.array([path[min(best_index + 1, len(path) - 1)].x, path[min(best_index + 1, len(path) - 1)].y])
        # avg_target_map = (p1 + p2) / 2.0
        avg_target_map = p1
        avg_target_base = self.convert_map_to_base(avg_target_map[0], avg_target_map[1])

        if (self.is_occupied(avg_target_base[0], avg_target_base[1])):
            print("target occupied")

        # debugging
        p_map = self.display_marker("map", 0.0, 0.0, 1.0, [avg_target_map[0], avg_target_map[1]])
        self.marker2_pub_.publish(p_map)

        # calculate curvature/steering angle
        # angle = (2 * avg_target_base[1]) / (self.lookahead_distance ** 2)
        angle = (2 * avg_target_base[1]) / (closest_distance_current_pose ** 2)
        angle = np.clip(angle, -np.deg2rad(self.clamp_angle), np.deg2rad(self.clamp_angle))
        angle = self.steering_gain * angle

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = self.get_speed(angle)

        print("steering at angle: ", np.rad2deg(angle))

        self.drive_pub_.publish(drive_msg)


    # The following methods are needed for RRT* and not RRT
    def cost(self, tree, node):
        """
        This method should return the cost of a node

        Args:
            node (Node): the current node the cost is calculated for
        Returns:
            cost (float): the cost value of the node
        """
        return tree[node.parent].cost + self.line_cost(tree[node.parent], node)


    def line_cost(self, n1, n2):
        """
        This method should return the cost of the straight line between n1 and n2

        Args:
            n1 (Node): node at one end of the straight line
            n2 (Node): node at the other end of the straint line
        Returns:
            cost (float): the cost value of the line
        """
        return np.sqrt((n1.x - n2.x)**2 + (n1.y - n2.y)**2)


    def near(self, tree, node):
        """
        This method should return the neighborhood of nodes around the given node

        Args:
            tree ([]): current tree as a list of Nodes
            node (Node): current node we're finding neighbors for
        Returns:
            neighborhood ([]): neighborhood of nodes as a list of Nodes
        """
        neighborhood = []
        for i in range(len(tree)):
            distance = (node.x - tree[i].x)**2 + (node.y - tree[i].y)**2
            if (distance < self.search_radius**2):
                neighborhood.append(i)

        return neighborhood

def main(args=None):
    rclpy.init(args=args)
    print("RRT Initialized")
    rrt_node = RRT()
    rclpy.spin(rrt_node)

    rrt_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
