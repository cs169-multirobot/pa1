#!/usr/bin/python

import sys
import rospy

from pa1.flock_member import FlockMember

if __name__ == "__main__":
    bot_id = int(sys.argv[1])
    flock_size = int(sys.argv[2])
    rospy.init_node("robot_" + str(bot_id) + "_flying", anonymous=False)

    bot = FlockMember(bot_id, flock_size)
    bot.spin()
