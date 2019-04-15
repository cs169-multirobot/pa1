#! /usr/bin/python

import rospy
import sys

from dynamic_reconfigure.server import Server
from pa1.cfg import FlockConfig

robot_id = 0

def config_callback(config, level):
    """
    Logs to screen the new configuration for the robot
    """
    rospy.loginfo("robot_" + str(robot_id) + " Reconfigure Request: {linear_speed}, {angular_speed}, {safe_distance}".format(**config))
    return config


if __name__ == "__main__":
    """
    Server for dynamically reconfiguring the robot's linear speed, angular speed, and safety distance 
    """
    robot_id = int(sys.argv[1])

    rospy.init_node("reconfig_flock_" + str(robot_id), anonymous=False)

    srv = Server(FlockConfig, config_callback)
    rospy.spin()
