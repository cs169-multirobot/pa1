#!/usr/bin/python

import sys
import rospy

from pa1.flock_member import FlockMember

if __name__ == "__main__":
    """
    Initialize robot and let it run its behaviors
    """
    robot_id = int(sys.argv[1])
    flock_size = int(sys.argv[2])

    rospy.init_node("robot_" + str(robot_id) + "_flying", anonymous=False)

    robot = FlockMember(robot_id, flock_size)
    robot.spin()
