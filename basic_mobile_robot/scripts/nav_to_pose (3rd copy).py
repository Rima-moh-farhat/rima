#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from std_msgs.msg import Int32MultiArray

class DijkstraPathPlanner(Node):
    def __init__(self):
        super().__init__('dijkstra_path_planner')
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        self.path_pub = self.create_publisher(
            Int32MultiArray,
            'path_topic',
            10
        )
        self.map = None

    def map_callback(self, msg):
        self.map = msg

    def find_path(self, start_point):
        if self.map is None:
            self.get_logger().error('Map has not been received yet.')
            return

        # Convert the start point to the map index
        start_index = self.point_to_index(start_point)

        # Create a distance array initialized with infinity
        distance = [float('inf')] * len(self.map.data)

        # Create a visited array initialized with False
        visited = [False] * len(self.map.data)

        # Create a previous array to store the previous node in the shortest path
        previous = [-1] * len(self.map.data)

        # Set the distance of the start point to 0
        distance[start_index] = 0

        # Dijkstra's algorithm
        for _ in range(len(self.map.data)):
            # Find the node with the minimum distance
            min_distance = float('inf')
            min_index = -1
            for i in range(len(self.map.data)):
                if not visited[i] and distance[i] < min_distance:
                    min_distance = distance[i]
                    min_index = i

            if min_index == -1:
                break

            visited[min_index] = True

            # Get the neighbors of the current node
            neighbors = self.get_neighbors(min_index)

            for neighbor in neighbors:
                if not visited[neighbor]:
                    # Calculate the distance to the neighbor
                    new_distance = distance[min_index] + self.get_distance(min_index, neighbor)

                    # Update the distance if it's smaller than the current distance
                    if new_distance < distance[neighbor]:
                        distance[neighbor] = new_distance
                        previous[neighbor] = min_index

        # Generate the path
        path = []
        current_index = self.point_to_index(start_point)
        while current_index != -1:
            current_point = self.index_to_point(current_index)
            path.append(current_point)
            current_index = previous[current_index]

        # Reverse the path
        path.reverse()

        # Publish the path
        path_msg = Int32MultiArray()
        path_msg.data = [self.point_to_index(point) for point in path]
        self.path_pub.publish(path_msg)

    def point_to_index(self, point):
        map_width = self.map.info.width
        map_origin_x = self.map.info.origin.position.x
        map_origin_y = self.map.info.origin.position.y
        resolution = self.map.info.resolution

        index_x = int((point.x - map_origin_x) / resolution)
        index_y = int((point.y - map_origin_y) / resolution)

        return index_y * map_width + index_x

    def index_to_point(self, index):
        map_width = self.map.info.width
        map_origin_x = self.map.info.origin.position.x
        map_origin_y = self.map.info.origin.position.y
        resolution = self.map.info.resolution

        x = index % map_width
        y = index // map_width

        point = Point()
        point.x = map_origin_x + (x + 0.5) * resolution
        point.y = map_origin_y + (y + 0.5) * resolution

        return point

    def get_neighbors(self, index):
        map_width = self.map.info.width
        map_height = self.map.info.height

        neighbors = []

        x = index % map_width
        y = index // map_width

        offsets = [(1, 0), (-1, 0), (0, 1), (0, -1)]

        for dx, dy in offsets:
            nx = x + dx
            ny = y + dy

            if 0 <= nx < map_width and 0 <= ny < map_height:
                neighbor_index = ny * map_width + nx
                if self.map.data[neighbor_index] != -1:
                    neighbors.append(neighbor_index)

        return neighbors

def get_distance(self, index1, index2):
    point1 = self.index_to_point(index1)
    point2 = self.index_to_point(index2)
    return math.sqrt((point1.x - point2.x) ** 2 +
                     (point1.y - point2.y) ** 2 +
                     (point1.z - point2.z) ** 2)
