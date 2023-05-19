#!/usr/bin/env python
import math # use of pi.
import numpy as np
import tf # library for transformations.
import rospy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist, PoseArray, Pose # message type for cmd_vel

FREQUENCY = 10 # Frequency at which the loop operates (Hz)
ROBOT_WIDTH = 0.25 # Robot size (~.2m for turtlebot)

ROBOT_GOAL_1 = (0.8,9) # DESTINATION
ROBOT_GOAL_2 = (1,1) # DESTINATION

# robot params that will be used
LINEAR_VELOCITY = .22 # m/s
ANGULAR_VELOCITY = math.pi/4 # rad/s

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

class PlanRobot:
    def __init__(self, linear_velocity=LINEAR_VELOCITY, angular_velocity=ANGULAR_VELOCITY):
        # set up publishers/subscribers/listeners
        self._map_sub = rospy.Subscriber('map', OccupancyGrid, self.map_callback, queue_size=1)
        self._poses_pub1 = rospy.Publisher("pose_sequence_1", PoseArray, queue_size=1)
        self._poses_pub2 = rospy.Publisher("pose_sequence_2", PoseArray, queue_size=1)
        self._cmd_pub1 = rospy.Publisher('/robot_0/cmd_vel', Twist, queue_size=1)
        self._cmd_pub2 = rospy.Publisher('/robot_1/cmd_vel', Twist, queue_size=1)
        self.listener = tf.TransformListener()
        
        # set up parameters
        self.map = None
        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity

    def map_callback(self, msg):
        self.map = Grid(msg.data, msg.info.width, msg.info.height, msg.info.resolution, msg.info.origin)

    def move(self, linear_vel, angular_vel, index):
        """Send a velocity command (linear vel in m/s, angular vel in rad/s)."""
        # Setting velocities.
        twist_msg = Twist()
        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel

        if index == 1:
            self._cmd_pub1.publish(twist_msg)
        else:
            self._cmd_pub2.publish(twist_msg)

    def stop(self, index):
        """Stop the robot."""
        twist_msg = Twist()
        if index == 1:
            self._cmd_pub1.publish(twist_msg)
        else:
            self._cmd_pub2.publish(twist_msg) 
    
    def rotate(self, angle, index):
        """Rotate for desired angle (angle in rad)."""
        # Setting rate
        rate = rospy.Rate(FREQUENCY)
        
        # rotate for desired duration
        rot_time = abs(angle / self.angular_velocity)
        start_time = rospy.get_rostime()
        while rospy.get_rostime() - start_time < rospy.Duration(rot_time):
            if angle > 0:
                self.move(0, self.angular_velocity, index)
            else:
                self.move(0, -self.angular_velocity, index)
            rate.sleep()
        self.stop(index)
    
    def forward(self, distance, index):
        """Move for desired distance (distance in m)."""
        # Setting rate
        rate = rospy.Rate(FREQUENCY)
        
        # move for desired duration
        move_time = distance / self.linear_velocity
        start_time = rospy.get_rostime()
        while rospy.get_rostime() - start_time < rospy.Duration(move_time):
            self.move(self.linear_velocity, 0, index)
            rate.sleep()
        self.stop(index)

    def get_pose_array(self, start, goal):
        """Return PoseArray for path from start to goal"""
        # obtain array of poses
        coordinates = self.bfs(start, goal) 
        
        # initialize PoseArray and header
        result = PoseArray() 
        result.header.stamp = rospy.Time.now()
        result.header.frame_id = "map"

        # for each coordinate, instantiate Pose object
        for i in range(len(coordinates)):
            pose = Pose()
            pose.position.x = coordinates[i][0] * self.map.resolution
            pose.position.y = coordinates[i][1] * self.map.resolution

            # calculate orientation to point towards next pose
            if i < len(coordinates) - 1:
                theta = math.atan2(coordinates[i+1][1] - coordinates[i][1], coordinates[i+1][0] - coordinates[i][0])
            quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)            
            pose.orientation.x = quaternion[0]
            pose.orientation.y = quaternion[1]
            pose.orientation.z = quaternion[2]
            pose.orientation.w = quaternion[3]

            # append Pose object to PoseArray
            result.poses.append(pose)
        return result

    def bfs(self, start, goal):
        """Use BFS to find path from start to goal"""
        """Returns sequence of poses in map reference frame"""
        # initialize variables
        DIR = [0, 1, 0, -1, 0]
        width = len(self.map.grid)
        height = len(self.map.grid[0])
        start = self.map.get_grid_coords(start) # convert start and goal to grid indicies
        goal = self.map.get_grid_coords(goal)
        result = []
        
        # initialize queue & tracking history for BFS
        frontier = [] # queue
        start_node = (start, None)
        frontier.append(start_node)
        explored = set() # history set
        explored.add(start)

        # iterate while queue is not empty
        while frontier:
            current_node = frontier.pop(0)
            current_state = current_node[0]

            # if reached desired goal
            if current_state == goal:
                # backtrack from current_node and return solution (in grid ref frame)
                while current_node is not None:
                    result.insert(0, current_node[0])
                    current_node = current_node[1]
                return result
            
            # iterate for each possible direction from current state
            for i in range(4):
                new_r = current_state[0] + DIR[i]
                new_c = current_state[1] + DIR[i+1]

                # check that new state is viable
                if new_r < 0 or new_r >= width \
                    or new_c < 0 or new_c >= height \
                    or self.map.cell_at(new_r, new_c) != 0 \
                    or (new_r, new_c) in explored:
                        continue
                
                # add child to queue & history set
                child = (new_r, new_c)
                explored.add(child)
                frontier.append((child, current_node))

        # if no path found, return empty list
        print("no path.")
        return result
    
    def follow_path(self, path, index):
        """Moves robot to follow given path (PoseArray)"""
        for i in range(len(path)):            
            # convert pose to be in terms of robot's frame (base_link)
            if index == 1:
                (trans, rot) = self.listener.lookupTransform('/robot_0/base_link', '/map', rospy.Time(0))
            else:
                (trans, rot) = self.listener.lookupTransform('/robot_1/base_link', '/map', rospy.Time(0))
            t = tf.transformations.translation_matrix(trans)
            R = tf.transformations.quaternion_matrix(rot)
            bl_T_map = t.dot(R)
            target_bl = bl_T_map.dot(np.array([[path[i].position.x], [path[i].position.y], [0], [1]]))

            # rotate robot to orient towards target
            theta = math.atan2(target_bl[1], target_bl[0])
            self.rotate(theta, index)

            # move robot towards target
            distance = math.sqrt(math.pow(target_bl[0], 2) + math.pow(target_bl[1], 2))
            self.forward(distance, index)
    
    def robot_to_goal(self, goal, index):
        """Finds path between current pose of robot and goal"""
        # get current pose of robot and find sequence of poses to get to the goal
        if index == 1:
            (trans, rot) = self.listener.lookupTransform('/map', '/robot_0/base_link', rospy.Time(0))
        else:
            (trans, rot) = self.listener.lookupTransform('/map', '/robot_1/base_link', rospy.Time(0))
        
        pose_array = self.get_pose_array((trans[0], trans[1]), goal)

        # publish a message containing the poses & move the robot 
        if index == 1:
            self._poses_pub1.publish(pose_array)
        else:
            self._poses_pub2.publish(pose_array)
        self.follow_path(pose_array.poses, index)
        

if __name__ == "__main__":
    rospy.init_node("planner")

    p = PlanRobot()

    rospy.sleep(2)

    # If interrupted, send a stop command before interrupting.
    #rospy.on_shutdown(p.stop(1))

    try:
        p.robot_to_goal(ROBOT_GOAL_1, 1)
        p.robot_to_goal(ROBOT_GOAL_2, 2)
        print("finished.")
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")

    rospy.spin()

