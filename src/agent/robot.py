import rospy
from enum import Enum, auto
from environment.inventory import OrderStatus
from .actions import Action
from .parts.moving import Moving
from .parts.vision import Vision
from .parts.communication import Communication
from .states.wander import Wander
from .states.begin_order import BeginOrder
from .states.bring_food import BringFood
from .states.end_order import EndOrder


class Robot():
    def __init__(self, restaurant, name="[ROBOT] "):
        """Initializes the robot.

        Args:
            restaurant (Restaurant): The restaurant object for requests
            name (str): The name of the robot (for logging)
        """
        # Assign restaurant and name
        self.restaurant = restaurant
        self.name = name

        # Init order list
        self.orders = [] 
        self.active_order = None
        
        # The initial robot state
        self.state = BeginOrder()

        # Init parts that make up robot
        self.moving = Moving(self.name)
        self.vision = Vision(self.name)
        self.communication = Communication(self.name)

        # Initialize the pose
        self.moving.init_pose()


    def update(self):
        """Updates the robot state."""
        self.state.update(self)

    
    def change_state(self, action):
        """Changes the current robot state.

        Args:
            action (Action): The flow action representing the next state
        """
        # Create new state based on FLOW
        if action is Action.FLOW.WANDER:
            self.state = Wander()
        elif action is Action.FLOW.BEGIN_ORDER:
            self.state = BeginOrder()
        elif action is Action.FLOW.BRING_FOOD:
            self.state = BringFood()
        elif action is Action.FLOW.END_ORDER:
            self.state = EndOrder()
        
        # Print out the information about new state transition
        rospy.loginfo(f"{self.name}Changed state to {action}")
    

    def end_order(self, order_id=None, success=True):
        """Finishes the order of the provided id.

        Args:
            order_id (int): The id of the order to be finished or `None`
            success (bool): Whether the order was finished successfully
        """
        # Get the order by ID from order list
        order = self.get_order_by_id(order_id)

        if order is self.active_order:
            # Remove the active order
            self.active_order = None

        # Change order status to `FINISHED` is its successful, else to `CANCELED`
        order.status = OrderStatus.FINISHED if success else OrderStatus.CANCELED
        
        if order.table is not None:
            # Assure to "unoccupy" the table if the table was assigned
            self.restaurant.set_table_occupancy(order.table.id, False)
        
        # Append order to history and del from list
        self.restaurant.order_history.append(order)
        self.orders.remove(order)

        # Print out that the id of the order that was finished and the remaining list
        rospy.loginfo(f"{self.name}Ended order {order.id}. Remaining: {self.orders}")
        rospy.loginfo(f"{self.name}Freed table {order.table.id}.")


    def get_order_by_id(self, order_id=None):
        """Gets the order object by its ID.

        Args:
            order_id (int): The ID of the requested order or `None`

        Returns:
            (Order): An order object whose ID is `order_id`
        """
        if order_id is None:
            # If order is None, take active
            order_id = self.active_order.id
        
        # Get the list of order IDs from the order list
        order_ids = [order.id for order in self.orders]

        return self.orders[order_ids.index(order_id)]

