import os
import time
import rospy

from time import time
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from move_base_msgs.msg import MoveBaseAction
from actionlib import SimpleActionClient
from utils.geom import euclidean_distance_poses, make_pose_cov, make_pose


class Moving():
    """ This class is used for the movement of the robot
    """
    LINEAR_VEL = 0.5
    ANGULAR_VEL = 2.0
    VEL_PUB_DURATION_LINEAR = 0.5  # Seconds
    VEL_PUB_DURATION_ANGULAR = 2.0  # Seconds
    MAX_RECENT = 50
    LINEAR_THRESHOLD = 0.01
    ANGULAR_THRESHOLD = 0.01
    XY_TOL = 0.01

    def __init__(self, basename=""):
        self.name = basename + "[MOVING] "
        self.initial_pose = make_pose_cov(0, -7)
        self.current_pose = make_pose(0, -7)
        self.status = None
        self.recovery_counter = 0
        self.recent_velocities = []
        self.recent_poses = []

        self.pose_publisher = rospy.Publisher(
            "/initialpose", PoseWithCovarianceStamped, queue_size=10)
        self.goal_publisher = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=10)
        self.status_subscriber = rospy.Subscriber(
            "/move_base/status", GoalStatusArray, self.status_callback, queue_size=1)
        self.amcl_subscriber = rospy.Subscriber(
            "/amcl_pose", PoseWithCovarianceStamped, self.robot_pose_callback, queue_size=10)
        self.vel_subscriber = rospy.Subscriber(
            "/cmd_vel", Twist, self.vel_callback, queue_size=10)

        rospy.sleep(5)

    def init_pose(self):
        """ Publishes to the /initialpose topic the robot's default initial pose,
         near the entrance
        """
        # Set the initial pose of the robot
        self.pose_publisher.publish(self.initial_pose)

    def status_callback(self, status):
        """ Get the status from the /move_base/status topic
        """
        self.status = status

    def robot_pose_callback(self, pose):
        """ Update the current_pose of the robot, as well as store the MAX_RECENT number of recent
         poses of the robot
        Args:
            pose(PoseWithCovarianceStamped): the current pose of the robot, from the /amcl_pose topic
        """
        self.current_pose = make_pose(
            pose.pose.pose.position.x, pose.pose.pose.position.y)
        self.recent_poses.append(self.current_pose)
        if len(self.recent_poses) > self.MAX_RECENT:
            self.recent_poses.pop(0)

    def get_status(self):
        """ Return the current robot's status of its navigation
        """
        if self.status != None and len(self.status.status_list) > 0:
            return int(self.status.status_list[-1].status)

        return GoalStatus.ABORTED

    def vel_callback(self, twist):
        """ Store the MAX_RECENT number of recent velocities of the robot
        Args:
            twist(Twist): the twist of the /cmd_vel topic
        """
        self.recent_velocities.append(twist)
        if len(self.recent_velocities) > self.MAX_RECENT:
            self.recent_velocities.pop(0)

    def cancel_path(self):
        """ Cancels the current path
        """
        client = SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()
        client.cancel_all_goals()
        rospy.loginfo("Cancelled goal_pose")

    def _move_lin(self, lin):
        """ Moves linearly
        Args:
            lin (float): the linear velocity
        """
        pub_velocity = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        twist = Twist()

        twist.linear.x = lin

        # Connect with the publisher
        while pub_velocity.get_num_connections() < 1:
            pass

        init_time = time()
        duration = 0
        # Publish to the /cmd_vel topic
        while duration < self.VEL_PUB_DURATION_LINEAR:
            pub_velocity.publish(twist)
            duration = time() - init_time

    def _move_ang(self, ang):
        """ Moves angularly
        Args:
            ang (float): the angular velocity
        """
        pub_velocity = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        twist = Twist()

        twist.angular.z = ang

        # Connect with the publisher
        while pub_velocity.get_num_connections() < 1:
            pass

        init_time = time()
        duration = 0
        # Publish to the /cmd_vel topic
        while duration < self.VEL_PUB_DURATION_ANGULAR:
            pub_velocity.publish(twist)
            duration = time() - init_time

    def move_forward(self):
        self._move_lin(self.LINEAR_VEL)

    def move_backward(self):
        self._move_lin(-self.LINEAR_VEL)

    def turn_left(self):
        self._move_ang(self.ANGULAR_VEL)

    def turn_right(self):
        self._move_ang(-self.ANGULAR_VEL)

    def recovery(self, pose):
        """ Cancels the current path, then moves around to get unstuck
        and then calls pub_goal_pose again to the same path
        Args:
            pose (): goal pose
        """
        rospy.loginfo(self.name + "Recovery Started")
        self.recovery_counter += 1
        self.cancel_path()

        self.move_forward()
        self.turn_right()
        self.move_forward()
        self.turn_left()

        rospy.loginfo(self.name + "Recovery Ended")
        self.goto_pose(pose)

    def _are_recent_vel_low(self):
        """ Checks if the recent_velocities are all low values
         (below LINEAR_THRESHOLD and ANGULAR_THRESHOLD)
        """
        if len(self.recent_velocities) == 0:
            return False
        elif len(self.recent_velocities) == self.MAX_RECENT:
            for i in range(len(self.recent_velocities)):
                linear_x = self.recent_velocities[i].linear.x
                angular_z = self.recent_velocities[i].angular.z
                if abs(linear_x) > self.LINEAR_THRESHOLD or \
                        abs(angular_z) > self.ANGULAR_THRESHOLD:
                    return False

        return True

    def _are_we_in_place(self):
        """ Checks if the robot has not moved, based on its recent_poses
        """
        if len(self.recent_poses) == 0:
            return False
        elif len(self.recent_poses) == self.MAX_RECENT:
            x_0 = self.recent_poses[0].pose.position.x
            y_0 = self.recent_poses[0].pose.position.y
            for i in range(1, len(self.recent_poses)):
                x = self.recent_poses[i].pose.position.x
                y = self.recent_poses[i].pose.position.y
                dist = euclidean_distance_poses(x_0, y_0, x, y)
                if dist > self.XY_TOL:
                    return False

        return True

    def goto_pose(self, pose):
        """ Goes to the provided pose
        Args:
            pose(PoseStamped): goal pose
        """

        def clear_costmaps():
            os.system('rosservice call /move_base/clear_costmaps \"{}\"')
            rospy.sleep(0.5)

        if self.recovery_counter > 2:
            rospy.logerr(self.name + "Could not get to location")
            return

        rospy.loginfo(self.name + "Departure - publishing to NavStack")
        clear_costmaps()
        self.goal_publisher.publish(pose)
        rospy.sleep(2)
        order_status = self.get_status()

        counter = 0
        self.recent_velocities = []
        while order_status != GoalStatus.SUCCEEDED:

            if order_status == GoalStatus.ABORTED:
                rospy.loginfo(self.name + "Unreachable location. Retrying.")
                clear_costmaps()
                self.recovery(pose)
            elif order_status != GoalStatus.ACTIVE:
                rospy.loginfo(f"{self.name}Order status: {order_status}")

            counter += 1

            if counter == 20:
                counter = 0
                clear_costmaps()

            rospy.sleep(0.2)
            order_status = self.get_status()

            if len(self.recent_velocities) >= self.MAX_RECENT and\
                    self._are_recent_vel_low() and \
                    self._are_we_in_place():
                self.recovery(pose)

        rospy.loginfo(self.name + "Arrival")
