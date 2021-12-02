
import random
from enum import Enum, auto
from ..robot import Status
from abstract_state import State
from ...util.geom import is_near, is_facing
from ..state_actions import Action
from .state_wander import Wander
from ...restaurant.order import Order, OrderStatus


class OrderBegin(State):
    """Robot is busy all the time until the state is finished.
    """
    class _SubState(Enum):
        def __init__(self):
            self.__super__()
            self.preference = False

        GOTO_ENTRANCE = auto()
        GREET_PEOPLE = auto()
        ASK_PREFERENCE = auto()
        FIND_TABLE = auto()
        GOTO_TABLE = auto()
        TELL_MENU = auto()
        TAKE_ORDER = auto()
        

    def __init__(self):
        self.substate = self._SubState.GOTO_ENTRANCE


    def _cancel_if_unclear(self, action, robot):
        if action is Action.BASE.REPEAT:
            # Generate sentences for rejecting the order
            sentence1 = "Are you literally that stupid?"
            sentence2 = "How many times do I have to ask the same question?"
            sentence3 = "Begone. I will not serve you today."

            # Say the sentences and cancel the order, switch to state WANDER
            robot.communication.say(f"{sentence1} {sentence2} {sentence3}")
            robot.orders[-1].status = OrderStatus.CANCELED
            robot.state = Action.FLOW.WANDER

            return True
        
        return False
    

    def _build_age_group_priority_queue(self, ages):
        pass


    def _get_table(self, available_tables, age_groups):
        pass


    def _single_customer_order(self, i, robot, options):
        while True:
            # Find out what the i'th customer would like to order today
            question = f"What would you like to order, customer {i+1}?"
            action, voice_data = robot.communication.ask(question, options)

            if self._cancel_if_unclear(action, robot):
                # Cancel order if not clear
                return Action.BASE.REJECT
            
            if voice_data in ["i will pass", "skip", "pass"]:
                # If the customer skips it
                return Action.BASE.IGNORE

            if voice_data in robot.restaurant.get_menu_n18() and \
                robot.orders[-1].people[i][1] <= 22:
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
                    robot.orders[-1].contents.append((voice_data, price))
            else:
                # If it's not from 18+ menu, just add it to list
                price = robot.restaurant.get_menu()[voice_data]
                robot.orders[-1].contents.append((voice_data, price))

            # Find out what the i'th customer would like to order today
            question = f"Would you like to order more, customer {i+1}?"
            action, voice_data = robot.communication.ask(question, binary=True)

            if self._cancel_if_unclear(action, robot):
                # Cancel order if not clear
                return Action.BASE.REJECT
            
            if not voice_data:
                # If nothing to add, end
                return Action.BASE.ACCEPT
        
        
        



    def do(self, robot):

        if self.substate is 1:
            # If robot has not yet reached the entrance
            self.substate = self.goto_entrance(robot)
        elif self.substate is 2:
            # If robot has not yet welcomed the people
            self.substate = self.greet_people(robot)
        elif self.substate is 3:
            # If robot has not yet found out the table
            self.substate = self.find_table(robot)
        elif self.substate is 4:
            # If robot has not yet guided the people
            self.substate = self.goto_table(robot)
        elif self.substate is 5:
            # If robot has not yet accepted the order
            self.substate = self.take_order(robot)


    def goto_entrance(self, robot):
        # Completions
        at_entrance = [
            is_near(robot.pos, robot.get_entrance_pos(), radius=20),
            is_facing(robot.pos, robot.heading, robot.get_entrance_pos())
        ]

        if not at_entrance[0]:
            # Move closer to the entrance position
            robot.move_to(robot.get_entrance_pos())
        
        if not at_entrance[1]:
            # Face more towards entrance position
            robot.face_to(robot.get_entrance_pos())
        
        return self.next(self.substate, all(at_entrance))
    

    def greet_people(self, robot):
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
        
        return self.next(self.substate)


    def find_table(self, robot):
        # Get num people to check possible tables
        num_people = len(robot.orders[-1].people)
        available_tables = robot.get_available_tables(num_people)

        if len(available_tables) == 1:
            # If there's only 1 available table, go to it
            robot.orders[-1].table = available_tables[0]
            return self._SubState.GOTO_TABLE

        # Ask for a table preference if there are >1
        question = "Do you have a table preference?"
        action, voice_data = robot.communication.ask(question, binary=True)

        if self._cancel_if_unclear(action, robot):
            # Cancel order if people play around
            return self._SubState.GOTO_ENTRANCE
        
        if voice_data:
            # Find out the table preference if answer was yes
            question = "Which table do you prefer to sit at?"
            options = [str(table.id) for table in available_tables]
            action, voice_data = robot.communication.ask(question, options)

            if self._cancel_if_unclear(action, robot):
                # Cancel order if people play around
                return self._SubState.GOTO_ENTRANCE
            
            # If the table ID is valid, assign the appropriate table to order
            robot.orders[-1].table = robot.get_table_by_id(int(voice_data))
        else:
            # Get the ages, find table priorities and assign a table
            ages = [person[1] for person in robot.orders[-1].people]
            age_groups = self._build_age_group_priority_queue(ages)
            robot.orders[-1].table = self._get_table(available_tables, age_groups)
        
        # Inform restaurant to update table availability
        robot.occupy_table(robot.orders[-1].table.id)

        return self.next(self.substate)


    def goto_table(self, robot):
        # Get table for current order
        table = robot.orders[-1].table

        # Completions
        at_table = [
            is_near(table.pos, robot.pos, radius=20),
            is_facing(robot.pos, robot.heading, table.pos)
        ]

        if not at_table[0]:
            # Move closer to table
            robot.move_to(table.pos)
        
        if not at_table[1]:
            # Face more towards table
            robot.face_to(table.pos)
        
        return self.next(all(at_table))
    

    def take_order(self, robot):
        # TODO: WAIT TILL POEPLE SIT DOWN BEFORE TELLING THE MENU speel(4)
        
        # Get the list of available menu items along with their prices 
        speech = [f"{k}: {v}" for k, v in robot.get_menu().items()]
        speech = '. '.join(speech)

        # Tell the menu to every customer and add options like "skip"/"pass"
        robot.communication.say(f"Let me now present you the menu. {speech}")
        options = robot.restaurant.get_menu().keys() + ["i will pass", "skip", "pass"]

        for i in len(robot.orders[-1].people):
            # Get the action to be performed after order execution
            action = self._single_customer_order(i, robot, options)
            
            if action is Action.BASE.REJECT:
                # If order not succesful, cancel it
                return self._SubState.GOTO_ENTRANCE
        
        # Finish taking the order and let the customer(s) know to wait for food
        robot.communication.say(f"I have accepted your order. Wait for food.")
        robot.orders[-1].status = OrderStatus.WAITING_FOOD
        robot.state = Action.FLOW.WANDER

        return self._SubState.GOTO_ENTRANCE
            
