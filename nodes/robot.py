#!/usr/bin/env python
import math # use of pi.
import numpy as np
import tf # library for transformations.
import rospy

from planning_package.src.grid import Grid
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist, PoseArray # message type for cmd_vel

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
