import math
import time
from geometry_msgs.msg import Point, Quaternion, PoseStamped, PoseWithCovarianceStamped


def _make_point_quaternion(x, y, theta):
    # Create point and quaternion
    point = Point(x=x, y=y, z=0)
    quaternion = Quaternion(x=0, y=0, z=0, w=1)

    if theta != 0:
        # Rotate the quaternion if theta angle provided
        quaternion = rotateQuaternion(quaternion, theta)
    
    return point, quaternion


def make_pose_cov(x, y, theta=0):
    # Create Covariance Pose, add meta
    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = "map"
    # pose.header.stamp.secs = time.time()

    # Get the position and the orientation based on x, y, theta
    position, orientation = _make_point_quaternion(x, y, theta)

    # Assign acquired pose attributes
    pose.pose.pose.position = position
    pose.pose.pose.orientation = orientation

    return pose


def make_pose(x, y, theta=0):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    # pose.header.stamp.secs = time.time()

    # Get the position and the orientation based on x, y, theta
    position, orientation = _make_point_quaternion(x, y, theta)

    # Assign the pose attributes
    pose.pose.position = position
    pose.pose.orientation = orientation
    
    return pose


def get_closest_pose(source_pose, target_poses):

    # Get source x y, initialize distances
    source_x = source_pose.pose.position.x
    source_y = source_pose.pose.position.y
    distances = {}

    for target_pose in target_poses:
        # Get target x, y and theta parameters
        target_x = target_pose.pose.position.x
        target_y = target_pose.pose.position.y
        target_theta = getHeading(target_pose.pose.orientation)
        
        # Calculate the euclidean distance from source to target, add to dict
        val = euclidean_distance_poses(source_x, source_y, target_x, target_y)
        distances[(target_x, target_y, target_theta)] = val

    # Get the entry with lowest euclid distance
    xytheta = min(distances, key=distances.get)

    return make_pose(xytheta[0], xytheta[1], xytheta[2])




def is_near(source_pose, target_pose, radius=1):
    x = source_pose.pose.position.x
    y = source_pose.pose.position.y
    x0 = target_pose.pose.position.x
    y0 = target_pose.pose.position.y

    return (x - x0) ** 2 + (y - y0) ** 2, radius ** 2, (x - x0) ** 2 + (y - y0) ** 2 < radius ** 2


def is_facing(source_pos, source_angle, target_pos):
    pass

def euclidean_distance_poses(x1, y1, x2, y2):
    return math.sqrt(math.pow(x1-x2, 2) + math.pow(y1-y2, 2))

def rotateQuaternion(q_orig, yaw):
    """
    Converts a basic rotation about the z-axis (in radians) into the
    Quaternion notation required by ROS transform and pose messages.
    
    :Args:
       | q_orig (geometry_msgs.msg.Quaternion): to be rotated
       | yaw (double): rotate by this amount in radians
    :Return:
       | (geometry_msgs.msg.Quaternion) q_orig rotated yaw about the z axis
     """
    # Create a temporary Quaternion to represent the change in heading
    q_headingChange = Quaternion()

    p = 0
    y = yaw / 2.0
    r = 0
 
    sinp = math.sin(p)
    siny = math.sin(y)
    sinr = math.sin(r)
    cosp = math.cos(p)
    cosy = math.cos(y)
    cosr = math.cos(r)
 
    q_headingChange.x = sinr * cosp * cosy - cosr * sinp * siny
    q_headingChange.y = cosr * sinp * cosy + sinr * cosp * siny
    q_headingChange.z = cosr * cosp * siny - sinr * sinp * cosy
    q_headingChange.w = cosr * cosp * cosy + sinr * sinp * siny

    # ----- Multiply new (heading-only) quaternion by the existing (pitch and bank) 
    # ----- quaternion. Order is important! Original orientation is the second 
    # ----- argument rotation which will be applied to the quaternion is the first 
    # ----- argument. 
    return multiply_quaternions(q_headingChange, q_orig)


def multiply_quaternions( qa, qb ):
    """
    Multiplies two quaternions to give the rotation of qb by qa.
    
    :Args:
       | qa (geometry_msgs.msg.Quaternion): rotation amount to apply to qb
       | qb (geometry_msgs.msg.Quaternion): to rotate by qa
    :Return:
       | (geometry_msgs.msg.Quaternion): qb rotated by qa.
    """
    combined = Quaternion()
    
    combined.w = (qa.w * qb.w - qa.x * qb.x - qa.y * qb.y - qa.z * qb.z)
    combined.x = (qa.x * qb.w + qa.w * qb.x + qa.y * qb.z - qa.z * qb.y)
    combined.y = (qa.w * qb.y - qa.x * qb.z + qa.y * qb.w + qa.z * qb.x)
    combined.z = (qa.w * qb.z + qa.x * qb.y - qa.y * qb.x + qa.z * qb.w)
    return combined


def getHeading(q):
    """
    Get the robot heading in radians from a Quaternion representation.
    
    :Args:
        | q (geometry_msgs.msg.Quaternion): a orientation about the z-axis
    :Return:
        | (double): Equivalent orientation about the z-axis in radians
    """
    yaw = math.atan2(2 * (q.x * q.y + q.w * q.z),
                     q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z)
    return yaw
