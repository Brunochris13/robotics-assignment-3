from enum import Enum, auto

class _BaseAction(Enum):
    ACCEPT = auto() # proceed to the next valid state
    REJECT = auto() # proceed to an alternative state
    REPEAT = auto() # repeat the previous state
    IGNORE = auto() # remove the previous state
    FINISH = auto() # terminate 

class _FlowAction(Enum):
    WANDER = auto()
    PHASE1 = auto() # meet people, guide to table, take order
    # might need a mid-phase for bringing food to ppl
    PHASE2 = auto() # goto table, calculate bill


class Action(Enum):
    BASE = _BaseAction()
    FLOW = _FlowAction()