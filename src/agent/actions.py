from enum import Enum, auto

class _BaseAction(Enum):
    ACCEPT = auto() # proceed to the next valid state
    REJECT = auto() # proceed to an alternative state
    REPEAT = auto() # repeat the previous state
    IGNORE = auto() # remove the previous state
    FINISH = auto() # terminate 

class _FlowAction(Enum):
    BEGIN_ORDER = auto()
    BRING_FOOD = auto()
    END_ORDER = auto()
    WANDER = auto()

class Action():
    BASE = _BaseAction
    FLOW = _FlowAction