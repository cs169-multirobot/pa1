import rospy
import numpy
import math

# message types for amcl_pose and base_scan topics, respectively
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from pa1.flock_math import *
from pa1.behaviors import command

import dynamic_reconfigure.client

SEPARATION = "separation"
ALIGNMENT = "alignment"
COHESION = "cohesion"

class FlockMember:
    def __init__(self, new_id, new_flock_size):
        self.id = new_id
        self.namespace = "/robot_" + str(self.id) + "/"
        self.flock_size = new_flock_size

        self.safe_distance = 1.0
        self.far_distance = 3.0
        self.linear_speed = 0.4
        self.angular_speed = 0.5

        self.behavior = SEPARATION

        self.cur_pose = (0.0, 0.0)
        self.flock_poses = []
        self.cur_orient = 0.0
        self.flock_orients = []

        # listen to dynamic reconfiguration of robot
        self.client = dynamic_reconfigure.client.Client("reconfig_flock_" + str(self.id), timeout=30, config_callback=self.config_callback)

        # initializing data structure to hold current pose and orientation of all robots in flock
        for bot_id in range(self.flock_size):
            self.flock_poses.append((0.0, 0.0))
            self.flock_orients.append(0.0)
            rospy.Subscriber("/robot_" + str(bot_id) + "/base_pose_ground_truth", Odometry, self.flock_pose_callback)

        rospy.Subscriber(self.namespace + "base_pose_ground_truth", Odometry, self.pose_callback) # current robot's position and orientation
        rospy.Subscriber(self.namespace + "base_scan", LaserScan, self.scan_callback) # current robot's laser scan readings

        self.cmd_pub = rospy.Publisher(self.namespace + "cmd_vel", Twist, queue_size=1)



    def config_callback(self, config):
        self.linear_speed = config['linear_speed']
        self.angular_speed = config['angular_speed']
        self.safe_distance = config['safe_distance']
        self.far_distance = self.safe_distance + 2.0


    def pose_callback(self, msg):
        self.cur_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.cur_orient = quat_to_euler(msg.pose.pose.orientation)


    def flock_pose_callback(self, msg):
        # retrieve robot's id
        bot = msg.header.frame_id
        extra, bot = bot.split("_", 1)
        bot, extra = bot.split("/", 1)
        bot_id = int(bot)

        # store robot's position and orientation for flocking behaviors
        self.flock_poses[bot_id] = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.flock_orients[bot_id] = quat_to_euler(msg.pose.pose.orientation)


    def scan_callback(self, msg):
        ranges = numpy.array(msg.ranges)
        min_laser_scan_reading = ranges.min()

        if min_laser_scan_reading < self.safe_distance:
            self.behavior = SEPARATION
        else:
            dist_from_flock = calc_dist_from_flock(self.flock_poses, self.flock_size, self.cur_pose)

            if dist_from_flock > self.far_distance:
                self.behavior = COHESION
            else:
                self.behavior = ALIGNMENT


    def spin(self):
        r = rospy.Rate(7)

        while not rospy.is_shutdown():

            avg_flock_pose = calc_avg_flock_pose(self.flock_poses, self.flock_size)
            avg_flock_orient = calc_avg_flock_orient(self.flock_orients, self.flock_size)

            linear_x, angular_z = command(self.behavior, self.linear_speed, self.angular_speed, self.cur_pose, avg_flock_pose, self.cur_orient, avg_flock_orient)

            cmd_msg = Twist()
            cmd_msg.linear.x = linear_x
            cmd_msg.angular.z = angular_z
            self.cmd_pub.publish(cmd_msg)

            r.sleep()
