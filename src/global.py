#!/usr/bin/env python
import math # use of pi.
import numpy as np
import tf # library for transformations.
import rospy
import threading

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist, PoseArray, Pose # message type for cmd_vel

FREQUENCY = 10 # Frequency at which the loop operates (Hz)
ROBOT_WIDTH = 0.25 # Robot size (~.2m for turtlebot)

# Initialize robots [(start), (goal)]
ROBOT_0 = [(2.0, 2.0), (0.8,9)] 
ROBOT_1 = [(5.0, 7.0), (1.0, 1.0)] 
ROBOT_2 = [(2.5, 5.0), (1.0, 3.0)] 
ROBOT_3 = [(4.0, 2.0), (3.0, 1.5)] 
ROBOT_4 = [(1.0, 8.0), (8.0, 5.0)] 
ROBOT_5 = [(1.5, 9.5), (9.0, 1.0)] 
ROBOT_6 = [(1.0, 4.0), (3.0, 4.0)] 

# robot params that will be used
LINEAR_VELOCITY = .22 # m/s
ANGULAR_VELOCITY = math.pi/4 # rad/s

class PlanRobots:
    def __init__(self, robots, linear_velocity=LINEAR_VELOCITY, angular_velocity=ANGULAR_VELOCITY):
        # set up subscriber
        self._map_sub = rospy.Subscriber('map', OccupancyGrid, self.map_callback, queue_size=1)
        self.map = None

        # instantiate robot object for each robot in environment
        self.robots = []
        for i in range(len(robots)):
            self.robots.append(Robot(str(i), robots[i][0], robots[i][1], linear_velocity, angular_velocity))        
    
    def stop(self):
        for robot in self.robots:
            robot.stop()

    def map_callback(self, msg):
        self.map = Grid(msg.data, msg.info.width, msg.info.height, msg.info.resolution, msg.info.origin)

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
            
            # CURRENTLY : marking cell as occupied if it is part of a robot's path
            self.map.mark_cell(coordinates[i][0], coordinates[i][1], 100)

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
    
    def move_robot_path(self, robot, path):
        # make robot move in desired path
        robot.follow_path(path)
    
    def path_plan(self):
        """Plan out path for each robot from start to goal"""
        """Execute path on robots"""

        threads = []
        for robot in self.robots:
            # obtain path from start to goal
            pose_array = self.get_pose_array(robot.start, robot.goal)
            # publish path 
            robot._poses_pub.publish(pose_array)
            # add function call to make robot move in the path
            threads.append(threading.Thread(target=self.move_robot_path, args=(robot, pose_array.poses)))
        
        # tell all the robots to start moving
        for thread in threads:
            thread.start()
        
        for thread in threads:
            thread.join()
        
class Robot:
    def __init__(self, number, start, goal, linear_velocity, angular_velocity):
        # set up publishers/subscribers/listeners
        self.listener = tf.TransformListener()
        self._map_sub = rospy.Subscriber('map', OccupancyGrid, self.map_callback, queue_size=1)
        self._poses_pub = rospy.Publisher("pose_sequence_" + number, PoseArray, queue_size=1)
        self._cmd_pub = rospy.Publisher("/robot_" + number + "/cmd_vel", Twist, queue_size=1)
        
        # set up parameters
        self.number = number
        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity
        self.start = start
        self.goal = goal
        self.map = None
    
    def map_callback(self, msg):
        self.map = Grid(msg.data, msg.info.width, msg.info.height, msg.info.resolution, msg.info.origin)

    def move(self, linear_vel, angular_vel):
        """Send a velocity command (linear vel in m/s, angular vel in rad/s)."""
        # Setting velocities.
        twist_msg = Twist()
        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        self._cmd_pub.publish(twist_msg)

    def stop(self):
        """Stop the robot."""
        twist_msg = Twist()
        self._cmd_pub.publish(twist_msg)
    
    def rotate(self, angle):
        """Rotate for desired angle (angle in rad)."""
        # Setting rate
        rate = rospy.Rate(FREQUENCY)
        
        # rotate for desired duration
        rot_time = abs(angle / self.angular_velocity)
        start_time = rospy.get_rostime()
        while rospy.get_rostime() - start_time < rospy.Duration(rot_time):
            if angle > 0:
                self.move(0, self.angular_velocity)
            else:
                self.move(0, -self.angular_velocity)
            rate.sleep()
        self.stop()
    
    def forward(self, distance):
        """Move for desired distance (distance in m)."""
        # Setting rate
        rate = rospy.Rate(FREQUENCY)
        
        # move for desired duration
        move_time = distance / self.linear_velocity
        start_time = rospy.get_rostime()
        while rospy.get_rostime() - start_time < rospy.Duration(move_time):
            self.move(self.linear_velocity, 0)
            rate.sleep()
        self.stop()
    
    def follow_path(self, path):
        """Moves robot to follow given path (PoseArray)"""
        for i in range(len(path)):            
            # convert pose to be in terms of robot's frame (base_link)
            (trans, rot) = self.listener.lookupTransform("/robot_" + self.number + "/base_link", '/map', rospy.Time(0))

            t = tf.transformations.translation_matrix(trans)
            R = tf.transformations.quaternion_matrix(rot)
            bl_T_map = t.dot(R)
            target_bl = bl_T_map.dot(np.array([[path[i].position.x], [path[i].position.y], [0], [1]]))

            # rotate robot to orient towards target
            theta = math.atan2(target_bl[1], target_bl[0])
            self.rotate(theta)

            # move robot towards target
            distance = math.sqrt(math.pow(target_bl[0], 2) + math.pow(target_bl[1], 2))
            self.forward(distance)

if __name__ == "__main__":
    rospy.init_node("planner")

    p = PlanRobots([ROBOT_0, ROBOT_1, ROBOT_2, ROBOT_3, ROBOT_4, ROBOT_5, ROBOT_6])

    rospy.sleep(2)

    # If interrupted, send a stop command before interrupting.
    rospy.on_shutdown(p.stop)

    try:
        p.path_plan()
        print("finished.")
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")

    rospy.spin()

