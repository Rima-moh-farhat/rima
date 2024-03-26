#!usr/bin/python3


resolution = 0.05000000074505806
width = 526
height = 268
origin = {
    "x": -resolution * width / 2.0,
    "y": -resolution * height / 2.0
}

def is_valid_move(x, y):
    if 0 <= x < width and 0 <= y < height:
        return True
    return False

def depth_first_search(start_x, start_y):
    stack = [(start_x, start_y)]

    while stack:
        x, y = stack.pop()

        for move in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            new_x = x + move[0]
            new_y = y + move[1]

            if is_valid_move(new_x, new_y):
                stack.append((new_x, new_y))

start_x = int(abs(origin["x"]) / resolution)
start_y = int(abs(origin["y"]) / resolution)
depth_first_search(start_x, start_y)

# Printing the map after exploration
for row in range(height):
    for col in range(width):
        if is_valid_move(col, row):
            print(" ", end="")
        else:
            print("#", end="")
    print()
