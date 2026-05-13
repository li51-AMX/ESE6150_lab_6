#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path

import numpy as np

class RRT_PP_Visualizer(Node):
    def __init__(self):
        super().__init__('rrt_pp_visualizer')

        # ===== Latest State =====
        self.target = None
        self.curvature = None
        self.latest_path = None
        self.latest_tree = None

        # ===== Subscriptions =====
        self.create_subscription(PointStamped, '/debug/target_wp', self.cb_target, 10)
        self.create_subscription(Float32, '/debug/target_curvature', self.cb_curv, 10)
        #self.create_subscription(Path, '/rrt_path', self.cb_path, 10)
        #self.create_subscription(Marker, '/rrt_tree', self.cb_tree, 10) # optional

        # ===== Publisher =====
        self.pub = self.create_publisher(Marker, '/rrt_pp_markers', 10)

        # ===== Continuous render loop =====
        self.create_timer(0.02, self.publish_all)

    # ============================================================
    # Callbacks (only store data)
    # ============================================================
    def cb_target(self, msg):
        self.target = msg

    def cb_curv(self, msg):
        self.curvature = msg.data

    def cb_path(self, msg):
        self.latest_path = msg

    def cb_tree(self, msg):
        self.latest_tree = msg

    # ============================================================
    # Main render loop (always uses latest data)
    # ============================================================
    def publish_all(self):
        if self.target is None or self.curvature is None:
            return

        self.publish_target()
        self.publish_arc()

        if self.latest_path is not None:
            self.publish_path(self.latest_path)

        if self.latest_tree is not None:
            self.publish_tree(self.latest_tree)

    # ============================================================
    # Visualization
    # ============================================================
    def publish_target(self):
        dot = Marker()
        dot.header = self.target.header
        dot.ns = "target"
        dot.id = 0
        dot.type = Marker.SPHERE
        dot.action = Marker.ADD

        dot.pose.position = self.target.point
        dot.pose.orientation.w = 1.0

        dot.scale.x = dot.scale.y = dot.scale.z = 0.25

        dot.color.a = 1.0
        dot.color.g = 1.0

        self.pub.publish(dot)

    def publish_arc(self):
        k = self.curvature

        arc = Marker()
        arc.header = self.target.header
        arc.ns = "arc"
        arc.id = 1
        arc.type = Marker.LINE_STRIP
        arc.action = Marker.ADD

        arc.scale.x = 0.05
        arc.color.a = 1.0
        arc.color.r = 1.0

        if abs(k) < 1e-6:
            self.pub.publish(arc)
            return

        R = 1.0 / abs(k)
        sign = np.sign(k)

        tx = self.target.point.x
        ty = self.target.point.y
        dist = np.hypot(tx, ty)

        theta_end = np.clip(dist / R, 0.0, np.pi / 2)

        for t in np.linspace(0, theta_end, 40):
            p = Point()
            p.x = R * np.sin(t)
            p.y = sign * R * (1 - np.cos(t))
            p.z = 0.0
            arc.points.append(p)

        self.pub.publish(arc)

    def publish_path(self, path_msg):
        marker = Marker()
        marker.header = path_msg.header
        marker.ns = "rrt_path"
        marker.id = 2
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.08
        marker.color.a = 1.0
        marker.color.b = 1.0

        for pose in path_msg.poses:
            p = Point()
            p.x = pose.pose.position.x
            p.y = pose.pose.position.y
            p.z = 0.0
            marker.points.append(p)

        self.pub.publish(marker)

    def publish_tree(self, tree_marker):
        tree_marker.ns = "rrt_tree"
        tree_marker.id = 3

        tree_marker.scale.x = 0.02
        tree_marker.color.a = 0.6
        tree_marker.color.r = 1.0

        self.pub.publish(tree_marker)

def main(args=None):
    rclpy.init(args=args)
    node = RRT_PP_Visualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()