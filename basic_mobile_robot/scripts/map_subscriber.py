#!/usr/bin/env python3

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile

def map_callback(msg):
    print(msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('map_subscriber')
    qos_profile = QoSProfile(depth=10)
    subscriber = node.create_subscription(
        OccupancyGrid,
        '/map',
        map_callback,
        qos_profile
    )
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
