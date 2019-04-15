cmake_minimum_required(VERSION 2.8.3)
project(pa1)

find_package(catkin REQUIRED
  rospy
  dynamic_reconfigure
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/flock.cfg
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
   scripts/init_world.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
   scripts/flock.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
