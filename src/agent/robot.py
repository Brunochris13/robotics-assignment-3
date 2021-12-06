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
        # Assign restaurant and name
        self.restaurant = restaurant
        self.name = name

        # Init order list
        self.orders = [] 
        self.active_order = None
        
        # Init the robot state
        self.state = Wander()

        # Init the robot parts
        self.moving = Moving(self.name)
        self.vision = Vision(self.name)
        self.communication = Communication(self.name)

        # Init waiter node and the pose
        # rospy.init_node("waiter_robot")
        self.moving.init_pose()


    def update(self):
        self.state.update(self)

    
    def change_state(self, action):
        if action is Action.FLOW.WANDER:
            self.state = Wander()
        elif action is Action.FLOW.BEGIN_ORDER:
            self.state = BeginOrder()
        elif action is Action.FLOW.BRING_FOOD:
            self.state = BringFood()
        elif action is Action.FLOW.END_ORDER:
            self.state = EndOrder()
        
        rospy.loginfo(f"{self.name}Changed state to {action}")
    

    def end_order(self, order_id=None, success=True):
        """Finishes the order of the provided id.

        Args:
            order_id (int): The id of the order to be cancelled
        """
        order = self.get_order_by_id(order_id)

        if order is self.active_order:
            self.active_order = None

        if success:
            order.status = OrderStatus.FINISHED
        else:
            order.status = OrderStatus.CANCELED
        
        self.restaurant.set_table_occupancy(order.table.id, False)
        self.restaurant.order_history.append(order)
        self.orders.remove(order)

        rospy.loginfo(f"{self.name}Ended order {order.id}. Remaining: {self.orders}")
        rospy.loginfo(f"{self.name}Freed table {order.table.id}.")


    def get_order_by_id(self, order_id=None):
        """Gets the order object by its ID.

        Args:
            order_id (int): The ID of the requested order

        Returns:
            (Order): An order object whose ID is `order_id`.
        """
        if order_id is None:
            order_id = self.active_order.id
        
        order_ids = [order.id for order in self.orders]
        return self.orders[order_ids.index(order_id)]

