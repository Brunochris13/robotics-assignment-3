import random
import rospy
from .inventory import Table
from utils.geom import make_pose, PI, PI_2
from waiter_robot.srv import Timer, TimerResponse
import threading
from time import sleep


INVENTORY = {
    "entrance": {"x": 0, "y": -6.8, "theta": -PI_2},
    "kitchen": {"x": 3, "y": 4.25, "theta": PI_2},
    "tables": [
        {"id": 1, "pose": {"position": {"x": -3.5, "y": -7, "z": 0}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 0}}, "num_people": 4, "type": "square"},
        {"id": 3, "pose": {"position": {"x": -4, "y": -3, "z": 0}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 0}}, "num_people": 6, "type": "rectangle"},
        {"id": 5, "pose": {"position": {"x": -3.5, "y": 1, "z": 0}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 0}}, "num_people": 4, "type": "square"},
        {"id": 7, "pose": {"position": {"x": -4, "y": 5, "z": 0}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 0}}, "num_people": 6, "type": "rectangle"},
        {"id": 2, "pose": {"position": {"x": 1.5, "y": -7, "z": 0}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 0}}, "num_people": 4, "type": "square"},
        {"id": 4, "pose": {"position": {"x": 1, "y": -3, "z": 0}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 0}}, "num_people": 6, "type": "rectangle"},
        {"id": 6, "pose": {"position": {"x": 1.5, "y": 1, "z": 0}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 0}}, "num_people": 4, "type": "square"}
    ]
}

# Menu (full)
MENU_ALL = {
    "hamburger": 10,
    "water": 3,
    "wine": 5,
}

# Menu (18+)
MENU_N18 = {
    "wine": 5,
}


LOCK = threading.Lock()


class Restaurant():
    """
    Note:
        Mainly methods should be used to update restaurnat attributes
        because multiple robots may connect to the same restaurant and
        updates may be pushed to external databases.
    """

    class _WaitingThread(threading.Thread):
        def __init__(self, restaurant, id, is_food, time=10):
            super().__init__()
            self.restaurant = restaurant
            self.id = id
            self.is_food = is_food
            self.time = time

        def run(self):
            type = "food" if self.is_food else "bill"
            rospy.loginfo(self.restaurant.name +
                          f"Order {self.id} started waiting for {type}")

            rospy.wait_for_service("timer")

            try:
                timer = rospy.ServiceProxy("timer", Timer)
                resp = timer(self.id, self.is_food, self.time)
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                resp = TimerResponse(self.id, self.is_food)

            with LOCK:
                self.restaurant.order_ids_ready.append((resp.id, resp.is_food))

            rospy.loginfo(self.restaurant.name +
                          f"Order {self.id} finished waiting for {type}")

    def __init__(self, name="[RESTAURANT] "):
        self.name = name
        self.entrance = make_pose(**INVENTORY["entrance"])
        self.kitchen = make_pose(**INVENTORY["kitchen"])
        self.tables = [
            Table(id=t["id"],
                  pose=make_pose(t["pose"]["position"]["x"],
                                 t["pose"]["position"]["y"]),
                  type=t["type"],
                  max_people=t["num_people"])
            for t in INVENTORY["tables"]
        ]

        self.order_ids_ready = []
        self.order_history = []

        rospy.Service("timer", Timer, self.handle_waiting)
        rospy.init_node("restaurant")

    def handle_waiting(self, req):
        sleep(req.time)
        return TimerResponse(req.id, req.is_food)

    def request_waiting(self, order_id, is_food=True, time=30):
        self._WaitingThread(self, order_id, is_food, time).start()

    def get_menu(self, n18=False):
        """Gets the restaurant menu with food names and prices.

        Args:
            n18 (bool): Whether to _only_ return products marked 18+

        Returns:
            (dict): A dictionary with menu items
        """
        return MENU_N18 if n18 else MENU_ALL

    def get_table_by_id(self, table_id):
        """Gets the table object by its ID.

        Args:
            table_id (int): The ID of the requested table

        Returns:
            (Table): A table object whose ID is `table_id`.
        """
        table_ids = [table.id for table in self.tables]
        return self.tables[table_ids.index(table_id)]

    def set_table_occupancy(self, table_id, occupied=True):
        """Sets the table as occupied (e.g. in database)

        Args:
            table_id (int): The ID of the table to change occupancy
            occupied (bool): Whether the table is occupied
        """
        self.get_table_by_id(table_id).occupied = occupied

    def get_available_tables(self, num_people=1):
        """Gets the available tables based on the number of people.

        Returns:
            (list(Table)): A list of available tables with enough space 
        """
        return [table for table in self.tables if not table.occupied and num_people <= table.max_people]

    def get_order_ids_ready(self, is_food):
        """Gets the list of order IDs for which the food/bill is ready.

        Returns:
            (list(int)): A list of order IDs with food/bill ready
        """
        return [order[0] for order in self.order_ids_ready if order[1] is is_food]

    def new_customer_exists(self):
        """Checks if there's any new customers at the entrance.

        Returns:
            (bool): Whether any new customer(s) arrived
        """
        return random.choice([True, False])
