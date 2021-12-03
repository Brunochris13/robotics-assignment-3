from enum import Enum, auto
from .abstract_state import State

class EndOrder(State):
    class _SubState(Enum):
        GOTO_TABLE = auto()
        ASK_IF_FINISHED = auto()
        RECEIVE_PAYMENT = auto()
    
    def __init__(self):
        self.substate = self._SubState.GOTO_TABLE
    
    def do(self, robot):
        if self.substate == 1:
            self.substate = self.goto_pos(robot, robot.active_order.table.pos)
        elif self.substate == 2:
            self.substate = self.ask_if_finished(robot)
        elif self.substate == 3:
            self.substate = self.receive_payment(robot)

    def ask_if_finished(self, robot):
        pass

    def receive_payment(self, robot):
        pass

