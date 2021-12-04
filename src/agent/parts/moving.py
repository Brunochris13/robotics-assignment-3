import os
import time
import rospy

from navigation.provide_goal_pose import pub_goal_pose
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from utils.geom import make_pose_cov, make_pose, is_near


class Moving():
    def __init__(self):
        self.mname = "[ROBOTCORE] "
        self.initial_pose = make_pose_cov(0, -7)
        self.current_pose = make_pose_cov(0, -7)
        self.status = None

        self.pose_publisher = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=10)
        self.goal_publisher = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.status_subscriber = rospy.Subscriber("/move_base/status", GoalStatusArray, self.status_callback, queue_size=1)
        self.amcl_subscriber = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.robot_pose_callback, queue_size=10)

        time.sleep(5)


    def init_pose(self):     
        # Set the initial pose of the robot
        self.pose_publisher.publish(self.initial_pose)
    

    def status_callback(self, status):
        self.status = status


    def robot_pose_callback(self, pose):
        self.current_pose = make_pose(pose.pose.pose.position.x, pose.pose.pose.position.y)


    def get_status(self):
        if self.status != None and len(self.status.status_list) > 0:
            return int(self.status.status_list[-1].status)
        
        return GoalStatus.ABORTED
    

    def goto_pose(self, pose):

        def set_goal():
            os.system('rosservice call /move_base/clear_costmaps \"{}\"')
            time.sleep(0.5)
            self.goal_publisher.publish(pose)

        print(self.mname + "Departure - publishing to NavStack")
        set_goal()
        time.sleep(2)
        order_status = self.get_status()
        
        counter = 0

        while order_status != GoalStatus.SUCCEEDED:

            d, r, b = is_near(self.current_pose, pose, radius=1.5)

            # print(d, r)

            if order_status == GoalStatus.ABORTED:
                print(self.mname + "Unreachable location. Retrying.")
                set_goal()
            elif order_status != GoalStatus.ACTIVE:
                print(f"Order status: {order_status}")
            
            counter += 1

            if counter == 20:
                counter = 0
                set_goal()
            
            time.sleep(0.2)
            order_status = self.get_status()

            #if b:
                # If near target position, publish self pose so it's automatically successful
            #    self.goal_publisher.publish(self.current_pose)

        print(self.mname + "Arrival")
    

    def move_to(self, target_pos):
        pub_goal_pose(target_pos.position.x, target_pos.position.y, 0)

    def turn_to(self, target_pos):
        pass