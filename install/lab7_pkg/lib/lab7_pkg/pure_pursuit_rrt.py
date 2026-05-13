#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PointStamped, PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import QoSProfile, DurabilityPolicy

from tf2_ros import Buffer, TransformListener

import numpy as np
import os, csv
#===============================================================================
#==================================== Setup ====================================
#===============================================================================
class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')
        # ===== Settings =====
        self.on_sim = True

        # ===== State variables =====
        self.speed = 0.0
        self.steering = 0.0
        self.waypoints = np.empty((0, 4))  # x, y, speed, lookahead

        # ===== Parameters =====
        param_defaults = {        
            'max_steering_angle': 0.4189,

            'speed_factor': 1.3,
            'lookahead_factor': 1.4,
            'curvature_p': 1.0,
        }

        for name, default in param_defaults.items():
            self.declare_parameter(name, default)
            setattr(self, name, self.get_parameter(name).value)

        self.param_cb_handle = self.add_on_set_parameters_callback(self._on_param_change)

        # ===== Subcriptions =====
        if(self.on_sim):
            self.pose_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.pose_callback, 10)
        else:
            self.pose_sub = self.create_subscription(Odometry, '/pf/pose/odom', self.pose_callback, 10)

        self.path_sub = self.create_subscription(Path, '/rrt_path', self.path_callback, 10)

        # ===== Publishers =====
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.goal_pub = self.create_publisher(PointStamped, '/debug/target_wp', 10)
        self.curvature_pub = self.create_publisher(Float32, '/debug/target_curvature', 10)
           
        # Setup tf listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    # ===== Parameter update =====
    def _on_param_change(self, params):
        for param in params:
            if hasattr(self, param.name):
                setattr(self, param.name, param.value)
                
        return SetParametersResult(successful=True)

    # ===== Load waypoints =====
    def path_callback(self, msg: Path):
        if len(msg.poses) == 0:
            self.get_logger().warn(f"Path message has no path")
            return
        if msg.header.frame_id != "map":
            self.get_logger().warn(f"Path frame is {msg.header.frame_id}, expected 'map'")

        points = []

        for i, pose in enumerate(msg.poses):
            x = pose.pose.position.x
            y = pose.pose.position.y

            # For now
            speed = 1.0
            lookahead = 0.7

            # Next
            # speed = self.compute_speed_from_curvature(...)
            # lookahead = self.compute_lookahead(...)

            points.append([x, y, speed, lookahead])

        self.waypoints = np.array(points)
        self.get_logger().info(f"Received RRT path: {len(points)} points")

#==================================================================================
#==================================== Main Loop ===================================
#==================================================================================
    def pose_callback(self, pose_msg):
        if self.waypoints.shape[0] == 0:
            self.get_logger().warn(f"No waypoints found")
            return

        # Get car world position
        car_pos = np.array([pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y])
        # Get target waypoint world position
        target_world = self.find_target_waypoint(car_pos)
        tx_world, ty_world, target_speed, lookahead = target_world

        # transform target waypoint to vehicle frame
        target_vehicle = self.manual_transform_to_vehicle_frame(pose_msg, [tx_world, ty_world])
        tx_vehicle, ty_vehicle = target_vehicle

        # Set steering and speed based on target
        curvature = self.compute_curvature(tx_vehicle, ty_vehicle)
        self.steering = np.clip(self.curvature_p * curvature,-self.max_steering_angle,self.max_steering_angle)
        self.speed = target_speed * self.speed_factor

        # Publish to topics
        self.publish_drive(self.steering, self.speed)
        self.publish_debug(tx_vehicle, ty_vehicle, curvature)

    # =============== Helper functions ===============
    def find_target_waypoint(self, car_pos):
        dists = np.linalg.norm(self.waypoints[:, :2] - car_pos, axis=1)
        closest_idx = np.argmin(dists)
        lookahead = self.waypoints[closest_idx, 3] * self.lookahead_factor

        acc_dist = 0.0
        idx = closest_idx

        while idx < len(self.waypoints) - 1:
            next_idx = idx + 1
            seg = np.linalg.norm(self.waypoints[next_idx, :2] - self.waypoints[idx, :2])
            acc_dist += seg

            if acc_dist >= lookahead:
                return self.waypoints[next_idx]

            idx = next_idx

        # If we reach the end → return last waypoint
        return self.waypoints[-1]

    def compute_curvature(self, x, y):
        Ld = np.hypot(x, y)
        if Ld < 1e-6:
            return 0.0
        return 2 * y / (Ld * Ld)

    def publish_drive(self, steering, speed):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.steering_angle = steering
        msg.drive.speed = speed
        self.drive_pub.publish(msg)

    def publish_debug(self, x, y, curvature):
        now = self.get_clock().now().to_msg()

        pt = PointStamped()
        
        if(self.on_sim):
            pt.header.frame_id = "ego_racecar/base_link"
        else:
            pt.header.frame_id = "base_link" # might be laser
        pt.header.stamp = now
        pt.point.x = float(x)
        pt.point.y = float(y)
        pt.point.z = 0.0
        self.goal_pub.publish(pt)

        curv = Float32()
        curv.data = float(curvature)
        self.curvature_pub.publish(curv)

    def manual_transform_to_vehicle_frame(self, pose_msg, waypoint):
        # Manual transform currently supports target_frame=base_link
        car_x = pose_msg.pose.pose.position.x
        car_y = pose_msg.pose.pose.position.y
        q = pose_msg.pose.pose.orientation
        # Convert quaternion to yaw and rotate world delta into vehicle frame.
        yaw = np.arctan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        dx = float(waypoint[0]) - car_x
        dy = float(waypoint[1]) - car_y
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)
        x_vehicle = cos_yaw * dx + sin_yaw * dy
        y_vehicle = -sin_yaw * dx + cos_yaw * dy

        return x_vehicle, y_vehicle

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
