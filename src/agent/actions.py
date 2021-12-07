from enum import Enum, auto

class _BaseAction(Enum):
    """An enum representing the base logic action an agent can take.

    This class has 5 different values. The guidelines for meanings are:
        * `ACCEPT`: proceed to the next valid state
        * `REJECT`: proceed to an alternative state
        * `REPEAT`: repeat the previous state
        * `IGNORE`: remove the previous state
        * `FINISH`: terminate
    
    Those are just guidelines, however. These actions can be used as
    helper values to monitor robot's success in performing tasks.
    """
    ACCEPT = auto()
    REJECT = auto()
    REPEAT = auto()
    IGNORE = auto()
    FINISH = auto()

class _FlowAction(Enum):
    """An enum representing the flow action a waiter robot can take.

    This class has 4 different values. Each enum represents a single
    big state the robot waiter can be in:
        * `BEGIN_ORDER`: the beginning of the order
        * `BRING_FOOD`: bringing food to customers
        * `END_ORDER`: the ending of the order
        * `WANDER`: free state where robot has nothing to do
    
    Each enum corresponds to a specific class in `states` package.
    """
    BEGIN_ORDER = auto()
    BRING_FOOD = auto()
    END_ORDER = auto()
    WANDER = auto()


class Action():
    """A class containing the actions the robot can perform.

    This class uses constants for the action types:
        `BASE`: represents the base action as in `_BaseAction`
        `FLOW`: represents the flow action as in `_FlowAction`

    This is the class that should be called to refer to a certain action
    the robot can take. It is convenient because the enums are organized
    and it shows the sub-enums belong only to the agent.
    """
    BASE = _BaseAction
    FLOW = _FlowAction