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

        # parameters that define speed and spacing of the robots
        # these will be updated dynamically
        self.safe_distance = 1.0
        self.far_distance = self.safe_distance + 3.0
        self.linear_speed = 0.4
        self.angular_speed = 0.5

        # current behavior of the robot
        self.behavior = SEPARATION

        # cur pose and orientation of the robot
        # initialized to 0 as a place holder
        self.cur_pose = (0.0, 0.0)
        self.cur_orient = 0.0

        # initializing data structure to hold current pose and orientation of all robots in flock
        self.flock_poses = []
        self.flock_orients = []
        for bot_id in range(self.flock_size):
            self.flock_poses.append((0.0, 0.0))
            self.flock_orients.append(0.0)
            rospy.Subscriber("/robot_" + str(bot_id) + "/base_pose_ground_truth", Odometry, self.flock_pose_callback)

        # listen to dynamic reconfiguration of robot
        self.client = dynamic_reconfigure.client.Client("reconfig_flock_" + str(self.id), timeout=30, config_callback=self.config_callback)

        rospy.Subscriber(self.namespace + "base_pose_ground_truth", Odometry, self.pose_callback) # current robot's position and orientation
        rospy.Subscriber(self.namespace + "base_scan", LaserScan, self.scan_callback) # current robot's laser scan readings

        self.cmd_pub = rospy.Publisher(self.namespace + "cmd_vel", Twist, queue_size=1)


    def config_callback(self, config):
        """
        Callback for dynamically reconfiguring the robot's parameters.

        Args:
            config: New configuration for the robot.
        """
        self.linear_speed = config['linear_speed']
        self.angular_speed = config['angular_speed']
        self.safe_distance = config['safe_distance']
        self.far_distance = self.safe_distance + 3.0


    def pose_callback(self, msg):
        """
        Callback for retrieving the robot's position and orientation.

        Args:
            msg: Odometry msg that has the current robot's position and orientation.
        """
        self.cur_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.cur_orient = quat_to_euler(msg.pose.pose.orientation)


    def flock_pose_callback(self, msg):
        """
        Callback for all robots to collect information of the flock's poses and orientaitons.

        Args:
            msg: Odometry msg describing one of the robot's pose and orientation.
        """
        # retrieve robot's id
        bot = msg.header.frame_id
        extra, bot = bot.split("_", 1)
        bot, extra = bot.split("/", 1)
        bot_id = int(bot)

        # store robot's position and orientation for flocking behaviors
        self.flock_poses[bot_id] = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.flock_orients[bot_id] = quat_to_euler(msg.pose.pose.orientation)


    def scan_callback(self, msg):
        """
        Callback for retrieving the robot's laser scan readings.
        If there is an obstacle near by, then the behavior should be separation.
        Else, the robot's behavior should be cohesion; robot needs to move to the flock's center.
        Or, the robot's behavior should be alignment; robot needs to keep the same heading as the flock's.
        It then finally published a command to implement the decided behavior.

        Args:
            msg: LaserScan msg describing the robot's laser scan readings.
        """
        ranges = numpy.array(msg.ranges)
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment

        # start off with the largest possible scan reading
        min_laser_scan_reading = msg.range_max

        # find the smallest laser scan reading in front of the robot
        for i in range(len(ranges)):
            if -math.pi / 3 < angle_min + angle_inc * i < math.pi / 3:
                if ranges[i] < min_laser_scan_reading:
                    min_laser_scan_reading = ranges[i]

        # start the separation behavior if there is an obstacle in the robot's safety area
        if min_laser_scan_reading < self.safe_distance:
            self.behavior = SEPARATION
        else:
            dist_from_flock = calc_dist_from_flock(self.flock_poses, self.flock_size, self.cur_pose)

            # robot needs to be relatively close to the flock
            if dist_from_flock > self.far_distance:
                self.behavior = COHESION
            # robot needs to follow the flock's heading
            else:
                self.behavior = ALIGNMENT


    def spin(self):
        """
        The robot should continously act in some behavior.
        """
        r = rospy.Rate(10)

        while not rospy.is_shutdown():

            avg_flock_pose = calc_avg_flock_pose(self.flock_poses, self.flock_size)
            avg_flock_orient = calc_avg_flock_orient(self.flock_orients, self.flock_size)

            linear_x, angular_z = command(self.behavior, self.linear_speed, self.angular_speed, self.cur_pose, avg_flock_pose, self.cur_orient, avg_flock_orient)

            # act corresponding to the current behavior of the robot
            cmd_msg = Twist()
            cmd_msg.linear.x = linear_x
            cmd_msg.angular.z = angular_z
            self.cmd_pub.publish(cmd_msg)

            r.sleep()
