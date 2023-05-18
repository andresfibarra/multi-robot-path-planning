#!/usr/bin/env python
#The line above is important so that this file is interpreted with Python when running it.

"""A grid class for the multi-robot path planning COSC81 final project.

For now, this is just the code from Andres's pa3. Just a baseline that we can add to. 

Date: May 18, 2023
"""

# Import of python modules.
import math # use of pi.

# import of relevant libraries.
import numpy as np
import rospy # module for ROS APIs
from nav_msgs.msg import Odometry # message type for odom
from nav_msgs.msg import OccupancyGrid

###
# CONSTANTS
###
ROBOT_SIZE = 0.178 # in meters


class Grid():
    def __init__(self, occupancy_grid_data, width, height, resolution):
        self.grid = np.reshape(occupancy_grid_data, (height, width))
        self.resolution = resolution
        self.width = width
        self.height = height
        self.expand_walls()
    
    def cell_at(self, x, y):
        """Returns contents of cell at position x, y in map frame"""
        return self.grid[y, x]

    def valid_point(self, row, col):
        """Returns if the cell is out of bounds."""
        if row < 0 or col < 0 or row >= self.height or col >= self.width:
            return False
        return True

    def expand_walls(self):
        """Virtually expand the walls so the robot won't crash."""
        expansion = int(ROBOT_SIZE / self.resolution)
        map_copy = np.copy(self.grid)
        for row in range(self.height):
            for col in range(self.width):
                if self.grid[row, col] != 0:
                    for r in range(-expansion, expansion+1):
                        for c in range(-expansion, expansion+1):
                            if self.valid_point(row + r, col + c):
                                map_copy[row + r, col + c] = 100
        self.grid = np.copy(map_copy)
