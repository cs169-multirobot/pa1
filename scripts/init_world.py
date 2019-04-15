#! /usr/bin/python
import os
import sys
import rospkg

import random

rospack = rospkg.RosPack()

initial_world_file = open(os.path.join(rospack.get_path("pa1"), "maps", "simple.world"), "r")
final_world_file = open(os.path.join(rospack.get_path("pa1"), "maps", "final.world"), "w")

final_world_file.write(initial_world_file.read())

robot_id = int(sys.argv[1])
for id in range(robot_id):
    final_world_file.write("\nturtlebot\n" +
        "(\n" +
        "   pose [ " + str(random.uniform(0.5, 9.5)) + " " + str(random.uniform(0.5, 9.5)) + " 0.0 0.0 ]\n" +
        "   name \"turtlebot" + str(id) + "\"\n" +
        "   color \"blue\"\n" +
        ")\n")

final_world_file.close()
initial_world_file.close()
