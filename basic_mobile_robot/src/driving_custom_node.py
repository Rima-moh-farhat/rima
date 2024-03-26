#!usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys

class DriverNode(Node):
    def __init__(self):
        super().__init__('driving_custom_node')
        self.publisher_ = self.create_publisher(Twist, '/basic_mobile_robot/cmd_vel', 10)
        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        linear_vel = 0.2  # سرعة خطية (قد تحتاج لتعديل هذه القيمة وفقًا لروبوتك)
        radius = 0.5  # شعاع الدوران (قد تحتاج لتعديل هذه القيمة وفقًا لروبوتك)
        msg.linear.x = linear_vel
        msg.linear.y = 0.0
        msg.angular.z = linear_vel / radius
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = DriverNode()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
