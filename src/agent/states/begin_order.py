import random
from enum import Enum, auto
from abstract_state import State
from ...util.geom import is_near, is_facing
from ..actions import Action
from ...restaurant.order import Order, OrderStatus
from .wander import Wander


class BeginOrder(State):
    """Robot is busy all the time until the state is finished.
    """
    class _SubState(Enum):
        GOTO_ENTRANCE = auto()
        MEET_PEOPLE = auto()
        FIND_TABLE = auto()
        GOTO_TABLE = auto()
        TAKE_ORDER = auto()


    def __init__(self):
        self.substate = self._SubState.GOTO_ENTRANCE


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

            # Say the sentences and cancel the order, switch to state Wander
            robot.communication.say(f"{sentence1} {sentence2} {sentence3}")
            robot.end_order(success=False)
            robot.state = Wander()

        return action is Action.BASE.REPEAT
    

    def _build_age_group_priority_queue(self, ages):
        """Builds a list of priority age groups.
        
        This method checks the given list of ages and constructs a list
        of age ranges in such order where the first element is the most
        likely range for the given ages.

        Args:
            ages (list(int)):
        
        Returns:
            (list(range)):
        """
        pass


    def _get_table_by_queue(self, available_tables, age_groups):
        """Gets the first possible table based on prioritized age group.

        Args:
            available_tables (list(Table)):
        
        Returns:
            (Table):
        """
        pass


    def _single_customer_order(self, i, robot, options):
        """Updates the robot's current order with customer's food order.

        Args:
            i (int): The number of the customer placing the order
            robot (Robot): The robot whose current order is updated
            options (list(str)): The list of available food choices

        Returns:
            (Action): An action indicating the success of placing order.
        """
        while True:
            # Find out what the i'th customer would like to order today
            question = f"What would you like to order, customer {i+1}?"
            action, voice_data = robot.communication.ask(question, options)

            if self._cancel_if_repeat(action, robot):
                # Cancel order if not clear
                return Action.BASE.REJECT
            
            if voice_data in ["i will pass", "skip", "pass"]:
                # If the customer skips it
                return Action.BASE.IGNORE

            if voice_data in robot.restaurant.get_menu(n18=True) and \
                robot.active_order.people[i][1] <= 22:
                # If the customer is suspected to be below 22, ask to confirm ID
                robot.communication.say("Please wait for staff to check your ID.")
                above_18 = input("The customer is above 18? (y/n)")
                above_18 = robot.communication.yesno_to_bool(above_18)

                if not above_18:
                    # If not above 18, don't add the requested order to list
                    robot.communication.say("Sorry, you cannot order that.")
                else:
                    # If above 18, confirm the order and add it to contents
                    robot.communication.say("Great, your ID was confirmed.")
                    price = robot.restaurant.get_menu()[voice_data]
                    robot.active_order.contents.append((voice_data, price))
            else:
                # If it's not from 18+ menu, just add it to list
                price = robot.restaurant.get_menu()[voice_data]
                robot.active_order.contents.append((voice_data, price))

            # Find out what the i'th customer would like to order today
            question = f"Would you like to order more, customer {i+1}?"
            action, voice_data = robot.communication.ask(question, binary=True)

            if self._cancel_if_repeat(action, robot):
                # Cancel order if not clear
                return Action.BASE.REJECT
            
            if not voice_data:
                # If nothing to add, end
                return Action.BASE.ACCEPT


    def do(self, robot):

        if self.substate is 1:
            # If robot has not yet reached the entrance
            self.substate = self.goto_pos(robot, robot.restaurant.entrance)
        elif self.substate is 2:
            # If robot has not yet welcomed the people
            self.substate = self.greet_people(robot)
        elif self.substate is 3:
            # If robot has not yet found out the table
            self.substate = self.find_table(robot)
        elif self.substate is 4:
            # If robot has not yet guided the people
            self.substate = self.goto_pos(robot, robot.active_order.table.pos)
        elif self.substate is 5:
            # If robot has not yet accepted the order
            self.substate = self.take_order(robot)
    

    def greet_people(self, robot):
        """Meets the people, scans their age and initiates a new order.

        Args:
            robot (Robot): The robot whose order to update
        """
        # Greeting messages
        AVAILABLE_GREETINGS = [
            "Greetings. I am Tob Taiwer and I will serve you today.",
        ]

        # Say a randomly chosen welcoming from the available ones
        robot.communication.say(random.choice(AVAILABLE_GREETINGS))

        # Get the available people
        people = robot.vision.see()

        # Begin the new order based on num people
        robot.orders.append(Order(people=people))
        robot.active_order = robot.orders[-1]
        
        return self.next(self.substate)


    def find_table(self, robot):
        """Asks for table preference and seats the people at the table.

        Args:
            robot (Robot): The robot whose order to update
        """
        # Get num people to check possible tables
        num_people = len(robot.active_order.people)
        available_tables = robot.restaurant.get_available_tables(num_people)

        if len(available_tables) == 0:
            # Cancel the order if there are no tables available to use
            robot.communication.say("Sorry, no more tables. Bye bye.")
            robot.end_order(success=False)
            robot.state = Wander()
            return self._SubState.GOTO_ENTRANCE

        if len(available_tables) == 1:
            # If there's only 1 available table, go to it
            robot.orders[-1].table = available_tables[0]
            return self._SubState.GOTO_TABLE

        # Ask for a table preference if there are >1
        question = "Do you have a table preference?"
        action, voice_data = robot.communication.ask(question, binary=True)

        if self._cancel_if_repeat(action, robot):
            # Cancel order if people play around
            return self._SubState.GOTO_ENTRANCE
        
        if voice_data:
            # Find out the table preference if answer was yes
            question = "Which table do you prefer to sit at?"
            options = [str(table.id) for table in available_tables]
            action, voice_data = robot.communication.ask(question, options)

            if self._cancel_if_repeat(action, robot):
                # Cancel order if people play around
                return self._SubState.GOTO_ENTRANCE
            
            # If the table ID is valid, assign the appropriate table to order
            robot.active_order.table = robot.restaurant.get_table_by_id(int(voice_data))
        else:
            # Get the ages, find table priorities and assign a table
            ages = [person[1] for person in robot.active_order.people]
            queue = self._build_age_group_priority_queue(ages)
            table = self._get_table_by_queue(available_tables, queue)
            robot.active_order.table = table
        
        # Inform restaurant to update table availability
        robot.restaurant.set_occupied_table(robot.active_order.table.id)

        return self.next(self.substate)
    

    def take_order(self, robot):
        """Tells the menu and takes the order from customers.

        Args:
            robot (Robot): The robot whose order to update
        """
        # TODO: WAIT TILL POEPLE SIT DOWN BEFORE TELLING THE MENU speel(4)
        
        # Get the list of available menu items along with their prices 
        speech = [f"{k}: {v} pounds" for k, v in robot.restaurant.get_menu().items()]
        speech = '. '.join(speech)

        # Tell the menu to every customer and add options like "skip"/"pass"
        robot.communication.say(f"Let me now present you the menu. {speech}")
        options = robot.restaurant.get_menu().keys() + ["i will pass", "skip"]

        for i in len(robot.current_order.people):
            # Get the action to be performed after order execution
            action = self._single_customer_order(i, robot, options)
            
            if action is Action.BASE.REJECT:
                # If order not succesful, cancel it
                return self._SubState.GOTO_ENTRANCE
        
        # Finish taking the order and let the customer(s) know to wait for food
        robot.communication.say(f"I have accepted your order. Wait for food.")
        robot.current_order.status = OrderStatus.WAITING_FOOD
        robot.state = Wander()

        return self._SubState.GOTO_ENTRANCE
            
