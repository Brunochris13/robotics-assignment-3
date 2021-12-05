from agent.actions import Action
import rospy
import random
from .abstract_state import State
from enum import Enum, auto
from utils.geom import make_pose, euclidean_distance_poses, getHeading, get_closest_pose

class Wander(State):
    class _SubState(Enum):
        WANDER = auto()
        SWITCH = auto()


    def __init__(self):
        self.substate = self._SubState.WANDER
    

    # def _closest_table_pose(self, robot, table):
    #     robot_x = robot.moving.current_pose.pose.position.x
    #     robot_y = robot.moving.current_pose.pose.position.y
    #     distances = {}

    #     for pose in table.robot_poses:
    #         table_x = pose.pose.position.x
    #         table_y = pose.pose.position.y
    #         table_theta = getHeading(pose.pose.orientation)
    #         distances[(table_x, table_y, table_theta)] = euclidean_distance_poses(robot_x, robot_y, table_x, table_y)

    #     xytheta = min(distances, key=distances.get)

    #     return make_pose(xytheta[0], xytheta[1], xytheta[2])


    def update(self, robot):
        """Simulates wandering and checks if it should switch state.

        Args:
            robot (Robot): The robot to move randomly
        """
        if self.substate is self._SubState.WANDER:
            self.substate = self.wander(robot)
        elif self.substate is self._SubState.SWITCH:
            self.substate = self.switch(robot)
    

    def wander(self, robot):
        """The robot is moved to some random position in the map.

        Args:
            robot (Robot): The robot to move to some random position
        """
        # Choose a random table to move to in the restaurant
        random_table = random.choice(robot.restaurant.tables)
        print(f"Approaching table {random_table.describe()}")

        robot_pose = robot.moving.current_pose
        table_poses = random_table.robot_poses
        closest_pose = get_closest_pose(robot_pose, table_poses)

        # ran_y = random.choice([-6, -5, -4])

        # random_pose = make_pose(0, ran_y)
        # print(f"Approaching (0, {ran_y})")

        # random_pose = random_table.pos
        # self.goto_pose(robot, random_pose)
        # self.goto_pose(robot, self._closest_table_pose(robot, random_table))

        return self.goto_pose(robot, closest_pose)
            

    def switch(self, robot):
        """Switches state if either of them is triggered.

        Args:
            robot (Robot): The robot whose state to switch
        """
        # Get the list of order IDs that belong to robot
        order_ids = [order.id for order in robot.orders]

        for order_id in []:#robot.restaurant.get_food_ready():
            if order_id in order_ids:
                robot.active_order = robot.orders[order_ids.index(order_id)]
                robot.change_state(Action.FLOW.BRING_FOOD)
                return self.prev(self.substate)
        
        for order_id in []:#robot.restaurant.get_bill_ready():
            if order_id in order_ids:
                robot.active_order = robot.orders[order_ids.index(order_id)]
                robot.change_state(Action.FLOW.END_ORDER)
                return self.prev(self.substate)
        
        if robot.restaurant.new_customer_exists():
            # If new customer has arrived, change state
            robot.change_state(Action.FLOW.BEGIN_ORDER)
        
        return self.prev(self.substate)
