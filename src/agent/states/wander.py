from agent.actions import Action
import rospy
import random
from .abstract_state import State
from utils.geom import make_pose, euclidean_distance_poses, getHeading

class Wander(State):
    def __init__(self):
        pass


    def update(self, robot):
        """Simulates wandering and checks if it should switch state.

        Args:
            robot (Robot): The robot to move randomly
        """
        self.wander(robot)
        # self.switch(robot)
    

    def wander(self, robot):
        """The robot is moved to some random position in the map.

        Args:
            robot (Robot): The robot to move to some random position
        """
        # Choose a random table to move to in the restaurant
        random_table = random.choice(robot.restaurant.tables)

        print(f"Approaching table {random_table.describe()}")

        # random_pose = random_table.pos
        # self.goto_pose(robot, random_pose)
        self.goto_pose(robot, self._closest_table_pose(robot, random_table))

    def _closest_table_pose(self, robot, table):
        robot_x = robot.moving.current_pose.pose.position.x
        robot_y = robot.moving.current_pose.pose.position.y
        distances = {}
        for pose in table.robot_poses:
            table_x = pose.pose.position.x
            table_y = pose.pose.position.y
            table_theta = getHeading(pose.pose.orientation)
            distances[(table_x, table_y, table_theta)] = euclidean_distance_poses(robot_x, robot_y, table_x, table_y)

        xytheta = min(distances, key=distances.get)
        return make_pose(xytheta[0], xytheta[1], xytheta[2])
            

    def switch(self, robot):
        """Switches state if either of them is triggered. 
        """
        order_ids = [order.id for order in robot.restaurant.orders]

        for order_id in robot.restaurant.get_food_ready():
            if order_id in order_ids:
                robot.active_order = robot.orders[order_ids.index(order_id)]
                robot.change_state(Action.FLOW.BRING_FOOD)
                return
        
        for order_id in robot.restaurant.get_bill_ready():
            if order_id in order_ids:
                robot.active_order = robot.orders[order_ids.index(order_id)]
                robot.change_state(Action.FLOW.END_ORDER)
                return
        
        if robot.restaurant.new_customer_exists():
            robot.state = robot.change_state(Action.FLOW.BEGIN_ORDER)
