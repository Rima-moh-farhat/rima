#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist

MOVEMENTS = {
    (-1, 0): "أعلى",
    (1, 0): "أسفل",
    (0, 1): "يمين",
    (0, -1): "يسار"
}

def get_neighbors(matrix, current):
    neighbors = []
    rows = len(matrix)
    cols = len(matrix[0])
    row, col = current

    if row > 0 and matrix[row - 1][col] != -1:
        neighbors.append(((row - 1, col), "أعلى"))
    if row < rows - 1 and matrix[row + 1][col] != -1:
        neighbors.append(((row + 1, col), "أسفل"))
    if col < cols - 1 and matrix[row][col + 1] != -1:
        neighbors.append(((row, col + 1), "يمين"))
    if col > 0 and matrix[row][col - 1] != -1:
        neighbors.append(((row, col - 1), "يسار"))

    return neighbors

def depth_first_search(matrix, start, goal):
    stack = [(start, [])]
    visited = set()

    while stack:
        current, path = stack.pop()
        if current == goal:
            return path
        if current not in visited:
            visited.add(current)
            neighbors = get_neighbors(matrix, current)
            for neighbor in neighbors:
                if neighbor[0] not in visited:
                    stack.append((neighbor[0], path + [neighbor[1]]))

    return None

def move_robot(direction):
    if direction == "أعلى":
        print("الروبوت يتحرك للأعلى.")
    elif direction == "أسفل":
        print("الروبوت يتحرك للأسفل.")
    elif direction == "يمين":
        print("الروبوت يتحرك لليمين.")
    elif direction == "يسار":
        print("الروبوت يتحرك لليسار.")

def main():
    matrix = [
        [1, -1, -1, -1, -1],
        [1,  0,  1,  0,  1],
        [1,  0,  1,  0,  1],
        [1,  0,  0,  0,  1],
        [1,  1,  1,  1,  1]
    ]
    start = (0, 0)  # نقطة البداية
    goal = (4, 4)  # الهدف

    path = depth_first_search(matrix, start, goal)

    if path:
        print("تم العثور على مسار:")
        print("المسار المحسوب:", path)
        for movement in path:
            move_robot(movement)
    else:
        print("لم يتم العثور على مسار.")

if __name__ == '__main__':
    rclpy.init()
    main()
    rclpy.shutdown()
