import rospy
import random
from utils.geom import euclidean_distance_poses
from enum import Enum, auto
from .abstract_state import State
from ..actions import Action
from environment.inventory import Order, OrderStatus


class BeginOrder(State):
    class _SubState(Enum):
        GOTO_ENTRANCE = auto()
        MEET_PEOPLE = auto()
        FIND_TABLE = auto()
        GOTO_TABLE = auto()
        TAKE_ORDER = auto()


    def __init__(self):
        self.substate = self._SubState.GOTO_ENTRANCE
    

    def _get_table_by_age(self, entrance, available_tables, ages):
        need_closest = all(map(lambda x: x >= 65, ages))
        table_poses = [table.pose for table in available_tables]

        if need_closest:
            x0, y0 = entrance.pose.position.x, entrance.pose.position.y
            calc_dist = lambda x: euclidean_distance_poses(x0, y0, x.x, x.y)
            distances = [calc_dist(table.pose.position) for table in table_poses]
            table = available_tables[distances.index(min(distances))]
        else:
            table = random.choice(available_tables)
        
        return table, need_closest


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

            if voice_data in robot.restaurant.get_menu(n18=True).keys() and \
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


    def update(self, robot):

        if self.substate is self._SubState.GOTO_ENTRANCE:
            # If robot has not yet reached the entrance
            self.substate = self.goto_pose(robot, robot.restaurant.entrance)
        elif self.substate is self._SubState.MEET_PEOPLE:
            # If robot has not yet welcomed the people
            self.substate = self.greet_people(robot)
        elif self.substate is self._SubState.FIND_TABLE:
            # If robot has not yet found out the table
            self.substate = self.find_table(robot)
        elif self.substate is self._SubState.GOTO_TABLE:
            # If robot has not yet guided the people
            # self.substate = self.goto_table(robot, robot.active_order.table)
            self.substate = self.next(self.substate)
        elif self.substate is self._SubState.TAKE_ORDER:
            # If robot has not yet accepted the order
            self.substate = self.take_order(robot)
    

    def greet_people(self, robot):
        """Meets the people, scans their age and initiates a new order.

        Args:
            robot (Robot): The robot whose order to update
        """
        # Greeting messages
        AVAILABLE_GREETINGS = [
            "Greetings. I will serve you today.",
            "Hello, let me be your waiter today."
        ]

        # Say a randomly chosen welcoming from the available ones
        robot.communication.say(random.choice(AVAILABLE_GREETINGS))

        # Get the available people
        # people = robot.vision.see()
        people = [(True, 32), (False, 29)]

        # Begin the new order based on num people
        robot.orders.append(Order(people=people))
        robot.active_order = robot.orders[-1]

        #
        rospy.loginfo(f"{robot.name}Started order with ID {robot.active_order.id}")
        
        return self.next(self.substate)


    def find_table(self, robot):
        """Asks for table preference and seats the people at the table.

        Args:
            robot (Robot): The robot whose order to update
        """
        # Get num people to check possible tables
        num_people = len(robot.active_order.people)
        available_tables = robot.restaurant.get_available_tables(num_people)

        print("Available tables:", [t.id for t in available_tables])

        if len(available_tables) == 0:
            # Cancel the order if there are no tables available to use
            robot.communication.say("Sorry, no more tables. Bye bye.")
            robot.change_state(Action.FLOW.WANDER)
            robot.end_order(success=False)
            return self._SubState.GOTO_ENTRANCE

        if len(available_tables) == 1:
            # If there's only 1 available table, go to it
            robot.active_order.table = available_tables[0]

            # Inform the restaurant to update the assigned table as occupied
            robot.restaurant.set_table_occupancy(robot.active_order.table.id)

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
            
            # If the table ID is valid, assign the appropriate table to the order object
            robot.active_order.table = robot.restaurant.get_table_by_id(int(voice_data))
        else:
            # Assign a random table or closest possible if it's senior
            ages = [person[1] for person in robot.active_order.people]
            table, closest = self._get_table_by_age(robot.restaurant.entrance, available_tables, ages)
            robot.active_order.table = table

            what = "closest possible" if closest else "random"
            rospy.loginfo(f"{robot.name}Occupied {what} table (ID: {table.id})")
        
        # Inform the restaurant to update the assigned table as occupied
        robot.restaurant.set_table_occupancy(robot.active_order.table.id)

        return self.next(self.substate)
    

    def take_order(self, robot):
        """Tells the menu and takes the order from customers.

        Args:
            robot (Robot): The robot whose order to update
        """        
        # Get the list of available menu items along with their prices 
        speech = [f"{k}: {v} pounds" for k, v in robot.restaurant.get_menu().items()]
        speech = '. '.join(speech)

        # Tell the menu to every customer and add options like "skip"/"pass"
        # robot.communication.say(f"Let me now present you the menu. {speech}")
        options = list(robot.restaurant.get_menu().keys()) + ["skip"]

        for i in range(len(robot.active_order.people)):
            # Get the action to be performed after order execution
            action = None#self._single_customer_order(i, robot, options)
            
            if action is Action.BASE.REJECT:
                # If order not succesful, cancel it
                return self._SubState.GOTO_ENTRANCE
        
        # Finish taking the order and let the customer(s) know to wait for food
        robot.communication.say(f"I have accepted your order. Wait for food.")
        robot.active_order.status = OrderStatus.WAITING_FOOD
        robot.change_state(Action.FLOW.WANDER)

        robot.restaurant.request_waiting(robot.active_order.id)

        return self._SubState.GOTO_ENTRANCE
            
