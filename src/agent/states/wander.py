from ..actions import Action
import rospy
import random
from .abstract_state import State
from enum import Enum, auto

class Wander(State):
    class _SubState(Enum):
        WANDER = auto()
        SWITCH = auto()


    def __init__(self):
        self.substate = self._SubState.WANDER


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

        return self.goto_table(robot, random_table)
            

    def switch(self, robot):
        """Switches state if either of them is triggered.

        Args:
            robot (Robot): The robot whose state to switch
        """
        # Get the list of order IDs that belong to robot
        order_ids = [order.id for order in robot.orders]

        for order_id in robot.restaurant.get_order_ids_ready(is_food=True):
            if order_id in order_ids:
                robot.active_order = robot.orders[order_ids.index(order_id)]
                robot.change_state(Action.FLOW.BRING_FOOD)
                return self.prev(self.substate)
        
        for order_id in robot.restaurant.get_order_ids_ready(is_food=False):
            if order_id in order_ids:
                robot.active_order = robot.orders[order_ids.index(order_id)]
                robot.change_state(Action.FLOW.END_ORDER)
                return self.prev(self.substate)
        
        if robot.restaurant.new_customer_exists():
            # If new customer has arrived, change state
            robot.change_state(Action.FLOW.BEGIN_ORDER)
        
        return self.prev(self.substate)
