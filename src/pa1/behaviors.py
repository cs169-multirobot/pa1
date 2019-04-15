import math

from pa1.flock_math import *

SEPARATION = "separation"
ALIGNMENT = "alignment"
COHESION = "cohesion"


def command(behavior, linear_speed, angular_speed, cur_pose, avg_flock_pose, cur_orient, avg_flock_orient):
    """
    Depending on the current behavior of the robot, it calls the corresponding behavior function.

    Args:
        behavior: Current behavior of the robot that needs to be implemented
        linear_spped: Current linear speed of the robot
        angular_speed: Current angular speed of the robot
        cur_pose: Current position of the robot; array formatted as [x, y]
        avg_flock_pose: Current center position of the flock; array formatted as [x, y]
        cur_orient: Current heading of the robot; it is the yaw of the orientation
        avg_flock_orient: Current avg heading of the flock; it is the yaw of the orientation
    Returns:
        Linear and angular speed for the behavior to be implemented
    """
    if behavior == SEPARATION:
        return separation(angular_speed)

    elif behavior == COHESION:
        return cohesion(linear_speed, angular_speed, cur_pose, avg_flock_pose, cur_orient)

    elif behavior == ALIGNMENT:
        return alignment(linear_speed, angular_speed, cur_orient, avg_flock_orient)


def separation(angular_speed):
    """
    Separation behavior: Turning away from obstacles.

    Args:
        angular_speed: Current angular speed of the robot
    Returns:
        Linear and angular speed for the behavior to be implemented
    """
    linear_x = 0.0
    angular_z = angular_speed

    return linear_x, angular_z


def cohesion(linear_speed, angular_speed, cur_pose, avg_flock_pose, cur_orient):
    """
    Cohesion behavior: Makes sure that the robot is heading towards the flock's center

    Args:
        linear_spped: Current linear speed of the robot
        angular_speed: Current angular speed of the robot
        cur_pose: Current position of the robot; array formatted as [x, y]
        avg_flock_pose: Current center position of the flock; array formatted as [x, y]
        cur_orient: Current heading of the robot; it is the yaw of the orientation
    Returns:
        Linear and angular speed for the behavior to be implemented
    """
    # calculate the angle of the robot to the center of the flock
    target_angle = math.atan2(avg_flock_pose[1] - cur_pose[1], avg_flock_pose[0] - cur_pose[0])
    # make sure the target_angle is in the correct format [-pi, pi]
    target_angle = rectify_angle_pi(target_angle)

    # check the sign to maybe help with more optimized turning
    error = target_angle - cur_orient
    if error > 0.0:
        sign = 1.0 # turn left
    else:
        sign = -1.0 # turn right

    # if the error is large, correct the heading of the robot
    if abs(error) > 0.2:
        linear_x = linear_speed
        angular_z = angular_speed * sign
    # the robot's heading is correct, continue forward
    else:
        linear_x = linear_speed
        angular_z = 0.0

    return linear_x, angular_z


def alignment(linear_speed, angular_speed, cur_orient, avg_flock_orient):
    """
    Alignment behavior: Makes sure that robot's heading is aligned with the flock's heading

    Args:
        linear_spped: Current linear speed of the robot
        angular_speed: Current angular speed of the robot
        cur_orient: Current heading of the robot; it is the yaw of the orientation
        avg_flock_orient: Current avg heading of the flock; it is the yaw of the orientation
    Returns:
        Linear and angular speed for the behavior to be implemented
    """
    # make sure the target_angle is in the correct format [-pi, pi]
    target_angle = rectify_angle_pi(avg_flock_orient)

    # check the sign to maybe help with more optimized turning
    error = target_angle - cur_orient
    if error > 0.0:
        sign = 1.0
    else:
        sign = -1.0

    # the error is large, correct the heading of the robot
    if abs(error) > 0.2:
        linear_x = linear_speed
        angular_z = angular_speed * sign
    # the robot's heading is correct, continue forward
    else:
        linear_x = linear_speed
        angular_z = 0.0

    return linear_x, angular_z
