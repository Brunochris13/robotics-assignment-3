import rospy
from time import time
from moving import *
from utils.geom import PI, PI_2, make_pose, getHeading, euclidean_distance_poses
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


class Test():

    def __init__(self):
        # Change this to True if you want error over time, for Noise Parameters and
        # Max/Min samples testing.
        # Do not forget to turn it on in moving.py as well
        self.test_amcl_params = False
        self.amcl_pose = make_pose(0, 0)
        self.ground_pose = make_pose(0, 0)

        rospy.Subscriber(
            "/amcl_pose", PoseWithCovarianceStamped, self.amcl_pose_callback, queue_size=10)
        rospy.Subscriber(
            "/base_pose_ground_truth", Odometry, self.ground_pose_callback, queue_size=10)

        mov = Moving()
        poses = [make_pose(0, 0,  0),
                 make_pose(-1, 2, PI),
                 make_pose(3, 4.5,  PI_2),
                 make_pose(2.5, -0.5,  -PI_2),
                 make_pose(0, -6.8,  -PI_2), ]

        mov.init_pose()

        for pose in poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            print(f"Navigation to ({x}, {y}):")

            init_time = time()
            if self.test_amcl_params:
                total_error = mov.goto_pose(pose)
            else:
                mov.goto_pose(pose)
            duration = time() - init_time

            xy_acc, yaw_acc = self.compare_poses(pose, self.ground_pose)

            print(f"Duration: {duration}")
            if self.test_amcl_params:
                print(f"Error Over Time: {total_error/duration}")
            else:
                print(f"Accuracy XY: {xy_acc}")
                print(f"Accuracy Yaw: {yaw_acc}")
            print()

    def compare_poses(self, pose1, pose2):
        x1 = pose1.pose.position.x
        y1 = pose1.pose.position.y
        x2 = pose2.pose.position.x
        y2 = pose2.pose.position.y

        eucl_dist = euclidean_distance_poses(x1, y1, x2, y2)

        yaw1 = getHeading(pose1.pose.orientation)
        yaw2 = getHeading(pose2.pose.orientation)

        yaw_diff = abs(yaw1-yaw2)

        return eucl_dist, yaw_diff

    def amcl_pose_callback(self, pose):
        self.amcl_pose = make_pose(
            pose.pose.pose.position.x, pose.pose.pose.position.y, getHeading(pose.pose.pose.orientation))

    def ground_pose_callback(self, pose):
        self.ground_pose = make_pose(
            pose.pose.pose.position.x, pose.pose.pose.position.y, getHeading(pose.pose.pose.orientation))


if __name__ == "__main__":
    rospy.init_node("test_navigation")
    test = Test()
