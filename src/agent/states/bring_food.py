from enum import Enum, auto
from .abstract_state import State
from ..actions import Action
from restaurant.order import OrderStatus

class BringFood(State):
    class _SubState(Enum):
        GOTO_KITCHEN = auto()
        GOTO_TABLE = auto()
        SERVE_PEOPLE = auto()
    
    def __init__(self):
        self.substate = self._SubState.GOTO_KITCHEN

    def update(self, robot):
        if self.substate is self._SubState.GOTO_KITCHEN:
            self.substate = self.goto_pose(robot, robot.restaurant.kitchen)
        elif self.substate is self._SubState.GOTO_TABLE:
            self.substate = self.goto_pose(robot, robot.active_order.table.pose)
        elif self.substate is self._SubState.SERVE_PEOPLE:
            self.substate = self.serve_people(robot)


    def serve_people(self, robot):
        """Serves food to people.

        Args:
            robot (Robot): The robot which serves food to people
        """
        # Inform people about the brought food, change order status and state
        robot.communication.say("Enjoy your meal. You have 4 seconds to eat.")
        robot.active_order.status = OrderStatus.WAITING_FOOD
        robot.change_state(Action.FLOW.WANDER)

        robot.restaurant.set_bill_waiting(robot.active_order.id)

        return self._SubState.GOTO_KITCHEN
        