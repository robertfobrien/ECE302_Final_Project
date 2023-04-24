import numpy as np
import cv2
import matplotlib.pyplot as plt
from collections import deque
import sys

curve = cv2.imread('curve.png', -1)
height, width = len(curve), len(curve[0])

# The start and end point you're looking at
start, end = (31, 14), (34, 51)

# All 8 directions
delta = [(-1, -1), (-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0), (1, -1), (0, -1)]

# Store the results of the BFS as the shortest distance to start
grid = [[sys.maxsize for _ in range(width)] for _ in range(height)]
grid[start[0]][start[1]] = 0

# The actual BFS algorithm
bfs = deque([start])
found = False
while len(bfs) > 0:
    y, x = bfs.popleft()
    # We've reached the end!
    if (y, x) == end:
        found = True
        break

    # Look all 8 directions for a good path
    for dy, dx in delta:
        yy, xx = y + dy, x + dx
        # If the next position hasn't already been looked at and it's white
        if 0 <= yy < height and 0 <= xx < width and grid[y][x] + 1 < grid[yy][xx] and curve[yy][xx] != 0:
            grid[yy][xx] = grid[y][x] + 1
            bfs.append((yy, xx))

if found:
    # Now rebuild the path from the end to beginning
    path = []
    y, x = end
    while grid[y][x] != 0:
        for dy, dx in delta:
            yy, xx = y + dy, x + dx
            if 0 <= yy < height and 0 <= xx < width and grid[yy][xx] == grid[y][x] - 1:
                path.append((yy, xx))
                y, x = yy, xx
    # Get rid of the starting point from the final path
    path.pop()

    # Display the final path on the plot
    fig, ax = plt.subplots()
    ax.scatter([start[1], end[1]], [start[0], end[0]], s=10, c='b')
    for y, x in path:
        ax.scatter(x, y, s=10, c='r')
    ax.imshow(curve, cmap='gray')

    plt.show()
else:
    print(f'No path found between {start} and {end}')