from abc import ABC, abstractmethod
from ..actions import Action


class State(ABC):
    def __init__(self):
        if type(self) is State:
            raise NotImplementedError("Cannot do direct instantiation!")
    
    
    @abstractmethod
    def update(self, robot):
        pass


    def _cancel_if_repeat(self, action, robot):
        """Cancels the order if the action is `REPEAT`.

        Args:
            action (Action): The action to be checked
            robot (Robot): The robot whose attributes to update
        
        Returns:
            (bool): True if action is `REPEAT` and False otherwise
        """
        if action is Action.BASE.REPEAT:
            # Generate sentences for rejecting the order
            sentence1 = "Are you literally that stupid?"
            sentence2 = "How many times do I have to ask the same question?"
            sentence3 = "Begone. I will not serve you today."

            # Say the sentences and cancel the order, switch to state WANDER
            robot.communication.say(f"{sentence1} {sentence2} {sentence3}")
            robot.change_state(Action.FLOW.WANDER)
            robot.end_order(success=False)

        return action is Action.BASE.REPEAT
    

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

    
    def goto_pose(self, robot, pose):
        """Moves robot to the assigned position.

        Args:
            robot (Robot): The robot whose position to update
            pose (PoseStamped): The position to move the robot to
        """
        # Move robot to given pose
        robot.moving.goto_pose(pose)
        
        return self.next(self.substate)
