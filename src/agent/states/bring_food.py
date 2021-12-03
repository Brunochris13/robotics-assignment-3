from enum import Enum, auto
from abstract_state import State
from wander import Wander

class BringFood(State):
    class _SubState(Enum):
        GOTO_KITCHEN = auto()
        GOTO_TABLE = auto()
        SERVE_PEOPLE = auto()
    
    def __init__(self):
        self.substate = self._SubState.GOTO_KITCHEN

    def do(self, robot):
        if self.substate is 1:
            self.substate = self.goto_pos(robot, robot.restaurant.kitchen)
        elif self.substate is 2:
            self.substate = self.goto_pos(robot, robot.active_order.table.pos)
        elif self.substate is 3:
            self.substate = self.serve_people(robot)


    def serve_people(self, robot):
        """Serves food to people.
        """
        robot.communication.say("Enjoy your meal. You have 4 seconds to eat.")
        robot.state = Wander()

        return self._SubState.GOTO_KITCHEN
        