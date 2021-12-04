import rospy
from msg import Timer

class State():
        def __init__(self, tableID, max):
            self.tableID = tableID
            self.maxTime = max
            self.time = 0
            self.done = False

        def next(self):
                self.time += 1

        def isDone(self):
                if self.maxTime == self.time:
                        return True
                else:
                        return False

def callback(self, msg):
        tableID = msg.tableID
        max = msg.time
        rospy.loginfo('tableID: {}, maxTime: {}'.format(tableID, max))
        
        state = State(tableID, max)
        states.append(state)

def main():


        for i in range(len(states)):
                state = states[i]
                state.next()
                if state.isDone():
                        state.done = True
                        t = Timer()
                        t.tableID = state.tableID
                        t.time = state.time
                        t.done = state.done
                        pub.publish(t)

        states = [state for state in states if state.done is False]





if __name__== '__main__':
        states = []
        rospy.init_node('restaurant')
        rate_interval = rospy.Rate(1)
        pub = rospy.Publisher('/timer', Timer, queue_size=10)
        sub = rospy.Subscriber("/timer", Timer, callback)

        while not rospy.is_shutdown():
                main()
                rate_interval.sleep()