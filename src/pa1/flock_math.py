import math
import tf
import sys


def calc_avg_flock_pose(flock_poses, flock_size):
    avg_x = 0.0
    avg_y = 0.0

    for pose in flock_poses:
        avg_x += pose[0]
        avg_y += pose[1]

    return ( avg_x / flock_size, avg_y / flock_size )


def calc_avg_flock_orient(flock_orients, flock_size):
    avg_yaw = 0.0

    for orient in flock_orients:
        avg_yaw += orient

    return avg_yaw / flock_size


def calc_dist_from_flock(flock_poses, flock_size, cur_pose):
    center = calc_avg_flock_pose(flock_poses, flock_size)
    distance = math.sqrt( math.pow(center[0] - cur_pose[0], 2) + math.pow(center[1] - cur_pose[1], 2) )

    return distance


def rectify_angle_2pi(angle):
    while angle < 0:
        angle += 2 * math.pi
    while angle > 2 * math.pi:
        angle -= 2 * math.pi
    return angle


def rectify_angle_pi(angle):
    angle = rectify_angle_2pi(angle)

    if angle > math.pi:
        angle -= 2 * math.pi

    return angle


def quat_to_euler(odom):
    quaternion = (
        odom.x,
        odom.y,
        odom.z,
        odom.w
    )
    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw = euler[2]
    return yaw
