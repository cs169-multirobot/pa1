#! /usr/bin/python

import rospy
import sys

from dynamic_reconfigure.server import Server
from pa1.cfg import FlockConfig


def config_callback(config, level):
    rospy.loginfo("""Reconfigure Request: {linear_speed}, {angular_speed}, {safe_distance}""".format(**config))
    return config


if __name__ == "__main__":
    bot_id = int(sys.argv[1])

    rospy.init_node("reconfig_flock_" + str(bot_id), anonymous=False)

    srv = Server(FlockConfig, config_callback)
    rospy.spin()
