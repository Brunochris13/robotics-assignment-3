import rospy
from nav_msgs.msg Odometry
from ..messages.order.msg Order

tables = []
# tables.append(<table id>, <x>, <y>)

class TableMonitor(object):
    def __init__(self, pub, tables):
        self._pub = pub
        self._tables = tables

    def callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        rospy.loginfo('x: {}, y: {}'.format(x, y))
        
        order = Order()
        order.tableID = random.random(x+y)
        self._pub.publish(order)

def main():
    rospy.init_node('restaurant_robot')
    
    # rospy.Publisher("<topic name>", <message class>, OPTION:<queue_size>)
    pub = rospy.Publisher('order', Order)
    monitor = TableMonitor(pub, tables)

    # rospy.Subscriber("<topic name>", <message class>, <callback>)
    rospy.Subscriber("/odom", Odometry, monitor.callback)
    
    rospy.spin()

if __name__=='__main__':
    main()