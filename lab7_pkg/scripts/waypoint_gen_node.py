#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import csv

class SimpleWaypointGen(Node):
    def __init__(self):
        super().__init__("simple_waypoint_gen")

        self.declare_parameter("output_file", "waypoints.csv")
        self.output_file = self.get_parameter("output_file").value

        self.points = []

        self.create_subscription(
            PoseWithCovarianceStamped,
            "/initialpose",
            self.callback,
            10
        )

        self.get_logger().info("Click poses to generate waypoints")

    def callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        self.points.append([x, y])
        self.get_logger().info(f"Added point: {x:.2f}, {y:.2f}")

    def interpolate(self, pts, spacing=0.5):
        pts = np.array(pts)
        new_pts = []

        for i in range(len(pts) - 1):
            p0, p1 = pts[i], pts[i + 1]
            vec = p1 - p0
            dist = np.linalg.norm(vec)

            if dist < 1e-6:
                continue

            steps = max(int(dist / spacing), 1)

            for j in range(steps):
                t = j / steps
                new_pts.append(p0 + t * vec)

        new_pts.append(pts[-1])
        return np.array(new_pts)

    def smooth(self, pts, window=5):
        if len(pts) < window:
            return pts

        smoothed = []
        half = window // 2

        for i in range(len(pts)):
            start = max(0, i - half)
            end = min(len(pts), i + half + 1)
            smoothed.append(np.mean(pts[start:end], axis=0))

        return np.array(smoothed)

    def save(self):
        if len(self.points) < 2:
            self.get_logger().info("Not enough points to save")
            return

        pts = self.interpolate(self.points)
        pts = self.smooth(pts)

        with open(self.output_file, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["x", "y"])

            for x, y in pts:
                writer.writerow([f"{x:.3f}", f"{y:.3f}"])

        self.get_logger().info(f"Saved {len(pts)} waypoints to {self.output_file}")


def main(args=None):
    rclpy.init(args=args)
    node = SimpleWaypointGen()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.save()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
