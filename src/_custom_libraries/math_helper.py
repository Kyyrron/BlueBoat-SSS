#!/usr/bin/env python3

from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Quaternion
from scipy.spatial.transform import Rotation as R
from typing import Tuple
import math
import numpy as np

def odometry(msg, quat = False):
    # Extract pose
    msg_pose = msg.pose.pose

    # Extract position
    x = msg_pose.position.x
    y = msg_pose.position.y
    z = msg_pose.position.z

    # Extract orientation (quaternion)
    qx = msg_pose.orientation.x
    qy = msg_pose.orientation.y
    qz = msg_pose.orientation.z
    qw = msg_pose.orientation.w

    if quat: # Use quaternion directly
        pose = [x,y,z,qx,qy,qz,qw]

    else:
        # Convert quaternion to roll, pitch, yaw
        rot = R.from_quat([qx, qy, qz, qw])
        roll, pitch, yaw = rot.as_euler('xyz', degrees=False)

        pose = [x,y,z,roll,pitch,yaw]

    # Extract twist
    twist = msg.twist.twist

    u = twist.linear.x
    v = twist.linear.y
    w = twist.linear.z

    p = twist.angular.x
    q = twist.angular.y
    r = twist.angular.z

    twist = [u,v,w,p,q,r]

    return pose, twist

def quaternion_multiply(q0, q1): # From https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Quaternion-Fundamentals.html
    """
    Multiplies two quaternions.

    Input
    :param q0: A 4 element array containing the first quaternion (q01, q11, q21, q31)
    :param q1: A 4 element array containing the second quaternion (q02, q12, q22, q32)

    Output
    :return: A 4 element array containing the final quaternion (q03,q13,q23,q33)

    """
    # Extract the values from q0
    w0 = q0[0]
    x0 = q0[1]
    y0 = q0[2]
    z0 = q0[3]

    # Extract the values from q1
    w1 = q1[0]
    x1 = q1[1]
    y1 = q1[2]
    z1 = q1[3]

    # Computer the product of the two quaternions, term by term
    q0q1_w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
    q0q1_x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
    q0q1_y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
    q0q1_z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1

    # Create a 4 element array containing the final quaternion
    final_quaternion = np.array([q0q1_w, q0q1_x, q0q1_y, q0q1_z])

    # Return a 4 element array containing the final quaternion (q02,q12,q22,q32)
    return final_quaternion

def quaternion_error(q2, q1): # Returns "q2-q1"
    q1[3] *= -1 # Negate for inverse

    return quaternion_multiply(q2, q1)

def quaternion_to_yaw(q: Quaternion):
    # yaw (Z axis rotation)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def yaw_to_quaternion(yaw: float):
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    return q

def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

def transform_body_to_world(x_r, y_r, yaw, x_b, y_b):
    c = math.cos(yaw)
    s = math.sin(yaw)

    x_w = x_r + (c * x_b - s * y_b)
    y_w = y_r + (s * x_b + c * y_b)

    return x_w, y_w

def enu_to_gps(lat0_deg, lon0_deg, east, north):
    EARTH_RADIUS = 6378137.0  # meters

    lat0 = math.radians(lat0_deg)
    lon0 = math.radians(lon0_deg)

    dlat = north / EARTH_RADIUS
    dlon = east / (EARTH_RADIUS * math.cos(lat0))

    lat = lat0 + dlat
    lon = lon0 + dlon

    return math.degrees(lat), math.degrees(lon)

def local_to_enu(x, y, yaw0):
    # rotate local frame into ENU
    theta = yaw0 - math.pi / 2.0

    c = math.cos(theta)
    s = math.sin(theta)

    east  = c * x - s * y
    north = s * x + c * y

    return east, north


def stamp_to_ns(stamp: TimeMsg) -> int:
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def quat_to_euler_rpy(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
    """Quaternion -> (roll, pitch, yaw) in radians (ZYX intrinsic, ROS convention)."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw
