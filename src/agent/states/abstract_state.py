from abc import ABC, abstractmethod

class State(ABC):
    def __init__(self):
        if type(self) is State:
            raise NotImplementedError("Cannot do direct instantiation!")
    
    @abstractmethod
    def do(self, robot):
        pass
    

    def next(self, substate, transit=True):
        # Get the enum members
        cls = substate.__class__
        members = list(cls)
        index = members.index(substate)

        if not transit:
            # If transit is False
            return members[index]

        if index + 1 >= len(members):
            # If next transition state does not exist, return
            raise StopIteration("End of enumeration reached")

        return members[index + 1]


    def prev(self, substate, transit=True):
        # Get the enum members
        cls = substate.__class__
        members = list(cls)
        index = members.index(substate)

        if not transit:
            # If transit is False
            return members[index]

        if index - 1 < 0:
            # If previous transition state does not exist, return
            raise StopIteration("Beginning of enumeration reached")
        
        return members[index - 1]
