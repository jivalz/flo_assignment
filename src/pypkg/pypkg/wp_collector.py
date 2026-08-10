#!/usr/bin/env python3
import math
import os 
import csv 
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry   

out = os.path.expanduser("~/assignment/src/pypkg/csv/waypoint.csv")

class wpcollector(Node):
    def __init__(self):
        super().__init__('waypoint_collector')
        self.declare_parameter("output_file", out)
        self.declare_parameter("min_distance",0.1)
        self.output_file = self.get_parameter("output_file").value
        self.min_distance = self.get_parameter("min_distance").value
        os.makedirs(os.path.dirname(self.output_file),exist_ok = True)
        self.file = open(self.output_file, 'w',newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(["id","x","y"])
        
        self.prev_x = None
        self.prev_y = None
        self.wp_count = 0 
        
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

    def odom_cb(self, msg:Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.prev_x is None or self.prev_y is None:
            self.record(x,y)
            return
        s = math.sqrt((x - self.prev_x)**2 + (y-self.prev_y)**2 )
        if s>=self.min_distance:
            self.record(x, y)

    def record(self, x, y):
        self.writer.writerow([self.wp_count, round(x, 4), round(y, 4)])
        self.file.flush()
        self.prev_x = x
        self.prev_y = y
        self.wp_count += 1
        #self.get_logger().info(f"Waypoint {self.wp_count:3d}: ({x:.3f}, {y:.3f})")

    def __del__(self):
        if hasattr(self, 'file') and not self.file.closed:
            self.file.close()
            print(f"\n{self.wp_count} waypoints saved to {self.output_file}")
            
def main():
    rclpy.init()
    node = wpcollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.__del__()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()



         


        



        

        