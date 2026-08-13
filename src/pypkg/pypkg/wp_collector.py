#!/usr/bin/env python3
import math
import os
import csv
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

try:
    from ament_index_python.packages import get_package_share_directory
    nav_share_dir = get_package_share_directory('nav')
    if '/install/' in nav_share_dir:
        ws_root = nav_share_dir.split('/install/')[0]
        DEFAULT_DIR = os.path.join(ws_root, 'src', 'nav', 'waypoints')
    else:
        DEFAULT_DIR = os.path.expanduser("~/flo_assignment/src/nav/waypoints")
except Exception:
    DEFAULT_DIR = os.path.expanduser("~/flo_assignment/src/nav/waypoints")

def get_next_waypoint_file(output_dir: str) -> str:
    os.makedirs(output_dir, exist_ok=True)
    index = 1
    while True:
        filepath = os.path.join(output_dir, f"waypoint{index}.csv")
        if not os.path.exists(filepath):
            return filepath
        index += 1

class WaypointCollector(Node):
    def __init__(self):
        super().__init__('waypoint_collector')
        self.declare_parameter("output_dir", DEFAULT_DIR)
        self.declare_parameter("file_name", "")
        self.declare_parameter("min_distance", 0.1)

        output_dir = os.path.expanduser(self.get_parameter("output_dir").value)
        file_name = self.get_parameter("file_name").value.strip()
        self.min_distance = self.get_parameter("min_distance").value

        if file_name:
            if not file_name.endswith('.csv'):
                file_name += '.csv'
            os.makedirs(output_dir, exist_ok=True)
            self.output_file = os.path.join(output_dir, file_name)
        else:
            self.output_file = get_next_waypoint_file(output_dir)

        self.file = open(self.output_file, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(["id", "x", "y"])

        self.prev_x = None
        self.prev_y = None
        self.wp_count = 0

        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.get_logger().info(f"Recording waypoints to: {self.output_file}")
        self.get_logger().info(f"Waypoint spacing threshold: {self.min_distance}m")

    def odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.prev_x is None or self.prev_y is None:
            self.record(x, y)
            return

        dist = math.hypot(x - self.prev_x, y - self.prev_y)
        if dist >= self.min_distance:
            self.record(x, y)

    def record(self, x: float, y: float):
        self.writer.writerow([self.wp_count, round(x, 4), round(y, 4)])
        self.file.flush()
        self.get_logger().info(f"Recorded Waypoint {self.wp_count}: ({x:.3f}, {y:.3f})")
        self.prev_x = x
        self.prev_y = y
        self.wp_count += 1
        #self.get_logger().info(f"Waypoint {self.wp_count:3d}: ({x:.3f}, {y:.3f})")

    def close(self):
        if hasattr(self, 'file') and not self.file.closed:
            self.file.close()
            self.get_logger().info(f"Saved {self.wp_count} waypoints to {self.output_file}")

def main(args=None):
    rclpy.init(args=args)
    node = WaypointCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()



         


        



        

        