import math

from pa1.flock_math import *

SEPARATION = "separation"
ALIGNMENT = "alignment"
COHESION = "cohesion"


def command(behavior, linear_speed, angular_speed, cur_pose, avg_flock_pose, cur_orient, avg_flock_orient):
    if behavior == SEPARATION:
        return separation(angular_speed)

    elif behavior == COHESION:
        return cohesion(linear_speed, angular_speed, cur_pose, avg_flock_pose, cur_orient)

    elif behavior == ALIGNMENT:
        return alignment(linear_speed, angular_speed, cur_orient, avg_flock_orient)


def separation(angular_speed):
    linear_x = 0.0
    angular_z = angular_speed

    return linear_x, angular_z


def cohesion(linear_speed, angular_speed, cur_pose, avg_flock_pose, cur_orient):
    angle = math.atan2(avg_flock_pose[1] - cur_pose[1], avg_flock_pose[0] - cur_pose[0])
    angle = rectify_angle_pi(angle)

    if angle > 0.0:
        sign = 1.0
    else:
        sign = -1.0

    if abs(angle - cur_orient) > 0.2:
        linear_x = linear_speed
        angular_z = angular_speed * sign
    else:
        linear_x = linear_speed
        angular_z = 0.0

    return linear_x, angular_z


def alignment(linear_speed, angular_speed, cur_orient, avg_flock_orient):
    target_angle = rectify_angle_pi(avg_flock_orient)

    error = target_angle - cur_orient
    if error > 0.0:
        sign = 1.0
    else:
        sign = -1.0

    if abs(error) > 0.1:
        linear_x = linear_speed
        angular_z = angular_speed * sign
    else:
        linear_x = linear_speed
        angular_z = 0.0

    return linear_x, angular_z
