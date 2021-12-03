from enum import Enum, auto
from abstract_state import State

class EndOrder(State):
    class _SubState(Enum):
        GOTO_TABLE = auto()
        ASK_IF_FINISHED = auto()
        RECEIVE_PAYMENT = auto()
    
    def __init__(self):
        self.substate = self._SubState.GOTO_TABLE
    
    def do(self, robot):
        pass

