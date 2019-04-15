#! /usr/bin/python
import os
import sys
import rospkg

import random

rospack = rospkg.RosPack()

# this world file has it all except the turtlebots
initial_world_file = open(os.path.join(rospack.get_path("pa1"), "maps", "simple.world"), "r")

# empty world file (or clears out previous info)
final_world_file = open(os.path.join(rospack.get_path("pa1"), "maps", "final.world"), "w")

# append simple.world to final.world
final_world_file.write(initial_world_file.read())

# add the turtlebots to final.world
robot_id = int(sys.argv[1])
for id in range(robot_id):
    final_world_file.write("\nturtlebot\n" +
        "(\n" +
        "   pose [ " + str(random.uniform(0.5, 9.5)) + " " + str(random.uniform(0.5, 9.5)) + " 0.0 0.0 ]\n" +
        "   name \"turtlebot" + str(id) + "\"\n" +
        "   color \"blue\"\n" +
        ")\n")

# close files
final_world_file.close()
initial_world_file.close()
