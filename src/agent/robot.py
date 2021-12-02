from enum import Enum
from enum import Enum, auto
from agent.parts.communication import Communication
from states.state_wander import Wander


class Status(Enum):
    AVAILABLE = auto()
    BUSY = auto()
    SPEAKING = auto()


class Robot():
    def __init__(self):
        self.orders = []
        self.status = Status.AVAILABLE
        self.state = Wander(self)
        self.communication = Communication()


    def update(self):
        pass


    def cont(self, callback, criteria, params1=[], params2=[]):
        if self.status == Status.BUSY:
            # Check if is has finished
            if not criteria(*params2):
                # If ended, go to the next task
                self.status = Status.AVAILABLE
                return True
        else:
            # Otherwise start the task
            self.status = Status.BUSY
            callback(*params1)
        
        return False

