import math
import tf
import sys


def calc_avg_flock_pose(flock_poses, flock_size):
    """
    Calculates the average x y coordinates of the flock.

    Args:
        flock_poses: Array of all of the robot's x y coordinates
        flock_size: Number of robots in the flock
    Returns:
        The avg x y coordinates of the flock.
    """
    avg_x = 0.0
    avg_y = 0.0

    for pose in flock_poses:
        avg_x += pose[0]
        avg_y += pose[1]

    return ( avg_x / flock_size, avg_y / flock_size )


def calc_avg_flock_orient(flock_orients, flock_size):
    """
    Calculates the average heading (yaw) of the flock.

    Args:
        flock_orients: Array of all of the robot's headings (yaw)
        flock_size: Number of robots in the flock
    Returns:
        The avg heading (yaw) of the flock.
    """
    avg_yaw = 0.0

    for orient in flock_orients:
        avg_yaw += orient

    return avg_yaw / flock_size


def calc_dist_from_flock(flock_poses, flock_size, cur_pose):
    """
    Calculates the distance from the robot to the flock's center.

    Args:
        flock_poses: Array of all of the robot's x y coordinates
        flock_size: Number of robots in the flock
        cur_pose: Current x y coordinates of the robot
    Returns:
        The avg heading (yaw) of the flock.
    """
    center = calc_avg_flock_pose(flock_poses, flock_size)
    distance = math.sqrt( math.pow(center[0] - cur_pose[0], 2) + math.pow(center[1] - cur_pose[1], 2) )

    return distance


def rectify_angle_2pi(angle):
    """
    Helper function for rectify_angle_pi.
    Adjusts angle to be in the range [0, 2pi]

    Args:
        angle: Angle that needs to be adjusted.
    Returns:
        The angle adjusted in the range [0, pi]
    """
    while angle < 0:
        angle += 2 * math.pi
    while angle > 2 * math.pi:
        angle -= 2 * math.pi
    return angle


def rectify_angle_pi(angle):
    """
    Adjusts the angle to be in the range [-pi, pi]

    Args:
        angle: Angle that needs to be adjusted.
    Returns:
        The angle adjusted in the range [-pi, pi]
    """
    # adjust angle to be in the range [0, 2pi]
    angle = rectify_angle_2pi(angle)

    # angles over pi should wrap around in the range [-pi, 0]
    if angle > math.pi:
        angle -= 2 * math.pi

    return angle


def quat_to_euler(odom):
    """
    Convery quaternion to euler angles.

    Args:
        odom: Orientation parameterized using quaternions.
    Returns:
        Orientation represented as yaw.
    """
    quaternion = (
        odom.x,
        odom.y,
        odom.z,
        odom.w
    )
    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw = euler[2]
    
    return yaw
