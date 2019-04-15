# Programming Assignment 1

Simulation of a flock of turtlebots that implements separation, cohesion, and alignment behaviors.

## Getting Started

This package has been tested on Ubuntu 16.04 and requires Stage.

Set up your caktin workspace:

```
mkdir -p catkin_ws/src
cd catkin_ws/src
```

### Install

Either move this package to the `src` folder in the catkin workspace or clone it from the source repository. Then build and source:

```
git clone https://github.com/cs169-multirobot/pa1.git
cd ..
catkin_make
```

### Run

To run the simulation (do not need to call `roscore`):

```
roslaunch pa1 flocking.launch [flock_size:=value...]
```

To dynamically reconfigure robot's parameters:

```
rosrun rqt_gui rqt_gui -s reconfigure
```
