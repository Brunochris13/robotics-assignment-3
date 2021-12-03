from enum import Enum, auto

from restaurant.order import OrderStatus
from .actions import Action
from restaurant.restaurant import Restaurant
from .parts.communication import Communication
from .parts.vision import Vision
from .parts.moving import Moving
from .states.wander import Wander
from .states.begin_order import BeginOrder
from .states.bring_food import BringFood
from .states.end_order import EndOrder


class Status(Enum):
    AVAILABLE = auto()
    BUSY = auto()


class Robot():
    def __init__(self):
        self.orders = []
        self.active_order = None
        self.state = Wander()
        self.moving = Moving()
        self.vision = Vision()
        self.communication = Communication()
        self.restaurant = Restaurant()


    def update(self):
        self.state.do(self)
        self.restaurant.update(self.orders)

    
    def change_state(self, action):
        if action is Action.FLOW.WANDER:
            self.state = Wander()
        elif action is Action.FLOW.BEGIN_ORDER:
            self.state = BeginOrder()
        elif action is Action.FLOW.BRING_FOOD:
            self.state = BringFood()
        elif action is Action.FLOW.END_ORDER:
            self.state = EndOrder()
    

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
        
        self.restaurant.order_history.append(order)
        self.orders.remove(order)


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


    """
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
    """

