from enum import Enum, auto

class _LogicAction(Enum):
    ACCEPT = auto() # proceed to the next valid state
    REJECT = auto() # proceed to an alternative state
    REPEAT = auto() # repeat the previous state
    IGNORE = auto() # remove the previous state
    FINISH = auto() # terminate 

class _BaseAction(Enum):
    SEE = auto()
    SPEAK = auto()
    LISTEN = auto()
    MOVE = auto()

class Action(Enum):
    LOGIC = _LogicAction()
    BASE = _BaseAction()