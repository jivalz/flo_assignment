import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class teleop(Node):
    def __init__(self):
        super().__init__('teleop')
        self.joy_sub = self.create_subscription(Joy,'joy',self.joycb,10)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel',10)
    def joycb(self,msg:Joy):
        vel = Twist()
        vel.linear.x = msg.axes[1]
        vel.angular.z = msg.axes[3]
        self.vel_pub.publish(vel)

def main(args=None):
    rclpy.init(args=args)
    node = teleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

