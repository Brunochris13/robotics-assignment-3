import random
from abstract_state import State
from begin_order import BeginOrder
from bring_food import BringFood
from end_order import EndOrder

class Wander(State):
    def __init__(self):
        pass


    def do(self, robot):
        """Wanders the robot and checks if it should switch state.
        """
        self.wander(robot)
        # self.switch(robot)
    

    def wander(self, robot):
        """The robot is moved to some random position in the map.
        """
        random_pos = random.choice([table.pos for table in robot.restaurant.tables])
        self.goto_pos(random_pos)


    def switch(self, robot):
        """Switches state if either of them is triggered. 
        """
        order_ids = [order.id for order in robot.orders]

        for order_id in robot.restaurant.get_food_ready():
            if order_id in order_ids:
                robot.active_order = robot.orders[order_ids.index(order_id)]
                robot.state = BringFood()
                return
        
        for order_id in robot.restaurant.get_bill_ready():
            if order_id in order_ids:
                robot.active_order = robot.orders[order_ids.index(order_id)]
                robot.state = EndOrder()
                return
        
        if robot.restaurant.new_customer_exists():
            robot.state = BeginOrder()
