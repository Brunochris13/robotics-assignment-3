from abc import ABC, abstractmethod
from utils.geom import is_near, is_facing

class State(ABC):
    def __init__(self):
        if type(self) is State:
            raise NotImplementedError("Cannot do direct instantiation!")
    
    @abstractmethod
    def do(self, robot):
        pass
    

    def next(self, substate, transit=True):
        """Given an enum, it retrieves the successor.

        Args:
            substate (Enum): The current substate enum
            transit (bool): Whether the transition is really needed
        
        Raises:
            (StopIteration): If there is no successor enum
        
        Returns:
            (Enum): Either current or successor substate enum
        """
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
        """Given an enum, it retrieves the precendent.

        Args:
            substate (Enum): The current substate enum
            transit (bool): Whether the transition is really needed
        
        Raises:
            (StopIteration): If there is no precendent enum
        
        Returns:
            (Enum): Either current or precendent substate enum
        """
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

    
    def goto_pos(self, robot, pos, radius=20):
        """Goes to the assigned position.

        Args:
            robot (Robot): The robot whose position to update
        """
        """
        # Completions
        at_pos = [
            is_near(pos, robot.pos, radius),
            is_facing(robot.pos, robot.heading, pos)
        ]

        if not at_pos[0]:
            # Move closer to table
            robot.moving.move_to(pos)
        
        if not at_pos[1]:
            # Face more towards table
            robot.moving.turn_to(pos)
        """

        robot.moving.move_to(pos)
        
        return self.next(self.substate)# self.next(self.substate, all(at_pos))
