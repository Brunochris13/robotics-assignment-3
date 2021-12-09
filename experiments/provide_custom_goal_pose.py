import rospy
from moving import *
from utils.geom import PI, PI_2, make_pose

if __name__ == "__main__":
    rospy.init_node("custom_goal_pose")
    mov = Moving()
    mov.init_pose()
    mov.goto_pose(make_pose(0,0,0))