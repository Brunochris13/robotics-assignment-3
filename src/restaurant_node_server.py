import os
import rospy
import sys
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0, parentdir)

from waiter_robot.srv import Timer, TimerResponse


# class State():
#         def __init__(self, tableID, max):
#             self.tableID = tableID
#             self.maxTime = max
#             self.time = 0
#             self.done = False

#         def next(self):
#                 self.time += 1

#         def isDone(self):
#                 return self.maxTime <= self.time
                

def callback(srv):
        ID = srv.ID
        time = srv.time
        rospy.loginfo('tableID: {}, maxTime: {}'.format(ID, time))
        rospy.sleep(time * 10)
        return TimerResponse(ID)
        

# def main(states):
#         for state in states:
#                 state.next()
#                 if state.isDone():
#                         state.done = True
#                         t = Timer()
#                         t.tableID = state.tableID
#                         t.time = state.time
#                         t.done = state.done
#                         pub.publish(t)
#         rospy.loginfo('len(states): {}'.format(len(states)))
#         states = [state for state in states if state.done is False]



if __name__== '__main__':
        states = []
        rospy.init_node('restaurant')
        rate_interval = rospy.Rate(1)
        server = rospy.Service('timer', Timer, callback)

        while not rospy.is_shutdown():
                rate_interval.sleep()