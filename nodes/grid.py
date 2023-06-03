#!/usr/bin/env python
#The line above is important so that this file is interpreted with Python when running it.

"""A grid class for the multi-robot path planning COSC81 final project.

Date: May 18, 2023
"""

# Import of python modules.
import math # use of pi.

# import of relevant libraries.
import numpy as np
import rospy # module for ROS APIs

# Constants
FREQUENCY = 10 # Frequency at which the loop operates (Hz)
ROBOT_WIDTH = 0.25 # Robot size (~.2m for turtlebot)

class Grid:
    def __init__(self, occupancy_grid_data, width, height, resolution, origin):
        self.grid = np.reshape(occupancy_grid_data, (height, width))
        self.resolution = resolution
        self.origin_pose = origin
        self.expand_walls()

    def get_grid_coords(self, coords):
        """Converts global coordinates to indicies for grid matrix"""
        return (int(coords[0] / self.resolution), int(coords[1] / self.resolution)) 

    def cell_at(self, x, y):
        return self.grid[y, x]
    
    def mark_cell(self, x, y, val):
        """Marks cell a desired value"""
        # currently being used to mark paths of other robots
        exp = int(ROBOT_WIDTH / self.resolution)
        expanded_grid = np.copy(self.grid)
        self.grid[y, x] = val
        # expand to prevent collisions
        for r_exp in range(-exp, exp+1):
            for c_exp in range(-exp, exp+1):
                new_r = y + r_exp
                new_c = x + c_exp
                if new_r >= 0 and new_r < len(self.grid) and new_c >= 0 and new_c < len(self.grid[0]):
                    expanded_grid[new_r, new_c] = 100
        self.grid = expanded_grid
    
    def expand_walls(self):
        """Implements expansion strategy to prevent wall collisions"""
        exp = int(ROBOT_WIDTH / self.resolution)
        expanded_grid = np.copy(self.grid)
        
        # for each cell, if a wall, then expand neighbors
        for row in range(len(self.grid)): 
            for col in range(len(self.grid[0])):
                if self.grid[row, col] != 0:
                    for r_exp in range(-exp, exp+1):
                        for c_exp in range(-exp, exp+1):
                            new_r = row + r_exp
                            new_c = col + c_exp
                            if new_r >= 0 and new_r < len(self.grid) and new_c >= 0 and new_c < len(self.grid[0]):
                                expanded_grid[new_r, new_c] = 100
        self.grid = expanded_grid