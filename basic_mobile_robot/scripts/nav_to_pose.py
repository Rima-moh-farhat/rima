#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from rclpy.qos import QoSProfile
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.task import Future
from threading import Thread
from queue import Queue
from geometry_msgs.msg import PoseStamped

MOVEMENTS = {
    'up': 'أعلى',
    'down': 'أسفل',
    'left': 'يسار',
    'right': 'يمين'
}

def get_neighbors(matrix, point):
    neighbors = []
    rows = len(matrix)
    cols = len(matrix[0])
    x, y = point

    if x > 0:
        neighbors.append((x - 1, y))  # Up
    if x < rows - 1:
        neighbors.append((x + 1, y))  # Down
    if y > 0:
        neighbors.append((x, y - 1))  # Left
    if y < cols - 1:
        neighbors.append((x, y + 1))  # Right

    return neighbors

def breadth_first_search(matrix, start, goal):
    visited = set()
    queue = Queue()
    queue.put([start])

    while not queue.empty():
        path = queue.get()
        current = path[-1]

        if current == goal:
            return path

        if current in visited:
            continue

        visited.add(current)

        for neighbor in get_neighbors(matrix, current):
            x, y = neighbor
            if matrix[x][y] != 100 and neighbor not in visited:
                new_path = list(path)
                new_path.append(neighbor)
                queue.put(new_path)

    return None

class MapSubscriber(Node):
    def __init__(self):
        super().__init__('map_subscriber')
        qos_profile = QoSProfile(depth=10)

        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            qos_profile
        )
        self.publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            qos_profile
        )

        self.goal_publisher = self.create_publisher(
            PoseStamped,
            'goal',
            qos_profile
        )

        self.navigation_action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=ReentrantCallbackGroup()
        )

    def move_robot(self, movement):
        twist = Twist()

        if movement == 'up':
            twist.linear.x = 1.0
        elif movement == 'down':
            twist.linear.x = -1.0
        elif movement == 'left':
            twist.angular.z = 1.0
        elif movement == 'right':
            twist.angular.z = -1.0

        self.publisher.publish(twist)

    async def map_callback(self, msg):
        matrix = msg.data
        width = msg.info.width
        height = msg.info.height

        # Convert 1D matrix to 2D matrix
        matrix = [matrix[i:i + width] for i in range(0, len(matrix), width)]

        start = (0, 0)
        goal = (height - 1, width - 1)

        path = breadth_first_search(matrix, start, goal)

        if path is None:
            self.get_logger().info('Unable to find a valid path.')
            return

        self.get_logger().info('Path: %s' % path)

        for point in path:
            map_subscriber.move_robot(MOVEMENTS['up'])

            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            pose.pose.orientation.w = 1.0

            future = Future()
            self.navigation_action_client.wait_for_server()
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = pose
            self.navigation_action_client.send_goal_async(goal_msg, feedback_callback=None)
            await self.navigation_action_client.future.result()

            while True:
                if self.navigation_action_client.future.done():
                    if self.navigation_action_client.future.result().result == 0:
                        break

        self.get_logger().info('Exploration completed.')

def main(args=None):
    rclpy.init(args=args),
    map_subscriber = MapSubscriber()

    executor = MultiThreadedExecutor()
    rclpy.spin(map_subscriber, executor=executor)

    map_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
