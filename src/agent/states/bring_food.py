from enum import Enum, auto
from .abstract_state import State
from ..actions import Action

class BringFood(State):
    class _SubState(Enum):
        GOTO_KITCHEN = auto()
        GOTO_TABLE = auto()
        SERVE_PEOPLE = auto()
    
    def __init__(self):
        self.substate = self._SubState.GOTO_KITCHEN

    def update(self, robot):
        if self.substate == 1:
            self.substate = self.goto_pose(robot, robot.restaurant.kitchen)
        elif self.substate == 2:
            self.substate = self.goto_pose(robot, robot.active_order.table.pos)
        elif self.substate == 3:
            self.substate = self.serve_people(robot)


    def serve_people(self, robot):
        """Serves food to people.
        """
        robot.communication.say("Enjoy your meal. You have 4 seconds to eat.")
        robot.change_state(Action.FLOW.WANDER)

        return self._SubState.GOTO_KITCHEN
        