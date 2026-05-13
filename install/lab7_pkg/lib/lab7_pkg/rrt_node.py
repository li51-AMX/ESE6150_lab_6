"""
This file contains the class definition for tree nodes and RRT
Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
"""
import numpy as np
from numpy import linalg as LA
import math
import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import OccupancyGrid

# TODO: import as you need

# class def for tree nodes
# It's up to you if you want to use this
class Node(object):
    def __init__(self):
        self.x = None
        self.y = None
        self.parent = None
        self.cost = None # only used in RRT*
        self.is_root = False

# class def for RRT
class RRT(Node):
    def __init__(self):
        # topics, not saved as attributes
        # TODO: grab topics from param file, you'll need to change the yaml file
        pose_topic = "ego_racecar/odom"
        scan_topic = "/scan"

        # you could add your own parameters to the rrt_params.yaml file,
        # and get them here as class attributes as shown above.

        # TODO: create subscribers
        self.pose_sub_ = self.create_subscription(
            PoseStamped,
            pose_topic,
            self.pose_callback,
            1)
        self.pose_sub_

        self.scan_sub_ = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            1)
        self.scan_sub_

        # publishers
        # TODO: create a drive message publisher, and other publishers that you might need

        # class attributes
        # TODO: maybe create your occupancy grid here
        occupancy_grid = np.array([])

    def scan_callback(self, scan_msg):
        """
        LaserScan callback, you should update your occupancy grid here

        Args: 
            scan_msg (LaserScan): incoming message from subscribed topic
        Returns:

        """

    def pose_callback(self, pose_msg):
        """
        The pose callback when subscribed to particle filter's inferred pose
        Here is where the main RRT loop happens

        Args: 
            pose_msg (PoseStamped): incoming message from subscribed topic
        Returns:

        """

        # start position
        start_x = pose_msg.pose.position.x
        start_y = pose_msg.pose.position.y
        start = (start_x, start_y)

        # fill in goal
        goal_x = 0
        goal_y = 0

        tree = [{"point": start, "parent": None}]
        path = None
        max_iter = 100

        for i in range(max_iter):
            sampled_point = self.sample()

            nearest_idx = self.nearest(tree, sampled_point)
            nearest_point = tree[nearest_idx]["point"]

            new_node = self.steer(nearest_point, sampled_point)

            if self.check_collision(nearest_point, new_node):
                tree.append({
                    "point": new_node,
                    "parent": nearest_idx
                })

                latest_added_node = len(tree) - 1
                if self.is_goal(new_node, goal_x, goal_y):
                    path = self.find_path(tree, latest_added_node)
                    break

        if path is not None:
            print("Path found:", path)
            self.path = path
        else:
            print("No path found")

    def sample(self):
        """
        This method should randomly sample the free space, and returns a viable point

        Args:
        Returns:
            (x, y) (float float): a tuple representing the sampled point

        """
        # array of the viable points (where the occupancy grid = 0)
        free_spaces = np.argwhere(self.occupancy_grid != 100)
        idx = np.random.randint(0, len(free_spaces))
        x, y = free_spaces[idx]
        return (x, y)

    def nearest(self, tree, sampled_point):
        """
        This method should return the nearest node on the tree to the sampled point

        Args:
            tree ([]): the current RRT tree
            sampled_point (tuple of (float, float)): point sampled in free space
        Returns:
            nearest_node (int): index of neareset node on the tree
        """
        min_dist = float('inf')
        nearest_node = 0
        sx, sy = sampled_point

        for i, node in enumerate(tree):
            x, y = node["point"]
            dist = (x - sx)**2 + (y - sy)**2
            
            if dist < min_dist:
                min_dist = dist
                nearest_node = i

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
        # Tune step size
        step_size = 1
        x_near = np.array(nearest_node)
        x_rand = np.array(sampled_point)

        vect = x_rand - x_near
        dist = np.linalg.norm(direction)

        # If the sampled point is within step_size, just go there
        if dist <= step_size:
            return tuple(x_rand)

        # Otherwise, move step_size toward it
        direction = vect / dist  # normalize
        new_node = x_near + step_size * direction

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
        x1, y1 = nearest_node
        x2, y2 = new_node

        dx = x2 - x1
        dy = y2 - y1
        dist = np.sqrt(dx**2 + dy**2)

        # how many points to check along the line
        steps = int(dist) + 1

        for i in range(steps + 1):
            t = i / steps
            x = x1 + t * dx
            y = y1 + t * dy

            row = int(round(y))
            col = int(round(x))

            # 1 id there is obstacle
            if self.occupancy_grid[row, col] == 100:
                return False
            
        return True

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
        # tune this
        goal_tolerance = 2.0

        x, y = latest_added_node
        dist = np.sqrt((x - goal_x)**2 + (y - goal_y)**2)

        return dist <= goal_tolerance

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

        while current is not None:
            path.append(tree[current]["point"])
            current = tree[current]["parent"]

        path.reverse()
        return path


    # The following methods are needed for RRT* and not RRT
    def cost(self, tree, node):
        """
        This method should return the cost of a node

        Args:
            node (Node): the current node the cost is calculated for
        Returns:
            cost (float): the cost value of the node
        """
        return 0

    def line_cost(self, n1, n2):
        """
        This method should return the cost of the straight line between n1 and n2

        Args:
            n1 (Node): node at one end of the straight line
            n2 (Node): node at the other end of the straint line
        Returns:
            cost (float): the cost value of the line
        """
        return 0

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