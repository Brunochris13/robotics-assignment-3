import math
import random
from geometry_msgs.msg import Pose, Point
from restaurant.table import Table
import rospy
# from restaurant.menu import menu
from .table import Table
from utils.geom import make_pose


# For faster calculations
PI = math.pi
PI_2 = PI/2

ITEMS = {
    "entrance": {"pose": {"position": {"x": 0, "y": -7, "z": 0}, "orientation": {"x": 0, "y": 0, "z": -0.7, "w": 0.7}}},
    "kitchen": {"pose": {"position": {"x": 3, "y": 4.5, "z": 0}, "orientation": {"x": 0, "y": 0, "z": 0.7, "w": 0.7}}},
    "centre": {"pose": {"position": {"x": 0, "y": 0, "z": 0}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 0}}},
    "tables": [
        {"id": 0, "pose": {"position": {"x": -3.5, "y": -7, "z": 0}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 0}}, "num_people": 4, "type": "square",
            "robot_pose": {"position_0": {"x": -4, "y": -6, "orientation": 0}, "position_1": {"x": -2.5, "y": -7.5, "orientation": PI_2},
                           "position_2": {"x": -1, "y": -6, "orientation": PI}, "position_3": {"x": -2.5, "y": -4.5, "orientation": -PI_2}}},
        {"id": 2, "pose": {"position": {"x": -4, "y": -3, "z": 0}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 0}}, "num_people": 6, "type": "rectangle",
         "robot_pose": {"position_0": {"x": -4.5, "y": -2, "orientation": 0}, "position_1": {"x": -2.5, "y": -3.5, "orientation": PI_2},
                        "position_2": {"x": -0.5, "y": -2, "orientation": PI}, "position_3": {"x": -2.5, "y": -0.5, "orientation": -PI_2}}},
        {"id": 4, "pose": {"position": {"x": -3.5, "y": 1, "z": 0}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 0}}, "num_people": 4, "type": "square",
         "robot_pose": {"position_0": {"x": -4, "y": 2, "orientation": 0}, "position_1": {"x": -2.5, "y": 0.5, "orientation": PI_2},
                        "position_2": {"x": -1, "y": 2, "orientation": PI}, "position_3": {"x": -2.5, "y": 3.5, "orientation": -PI_2}}},
        {"id": 6, "pose": {"position": {"x": -4, "y": 5, "z": 0}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 0}}, "num_people": 6, "type": "rectangle",
         "robot_pose": {"position_0": {"x": -4.5, "y": 6, "orientation": 0}, "position_1": {"x": -2.5, "y": 4.5, "orientation": PI_2},
                        "position_2": {"x": -0.5, "y": 6, "orientation": PI}, "position_3": {"x": -2.5, "y": 7.5, "orientation": -PI_2}}},
        {"id": 1, "pose": {"position": {"x": 1.5, "y": -7, "z": 0}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 0}}, "num_people": 4, "type": "square",
        "robot_pose": {"position_0": {"x": 1, "y": -6, "orientation": 0}, "position_1": {"x": 2.5, "y": -7.5, "orientation": PI_2},
                       "position_2": {"x": 4, "y": -6, "orientation": PI}, "position_3": {"x": 2.5, "y": -4.5, "orientation": -PI_2}}}, 
        {"id": 3, "pose": {"position": {"x": 1, "y": -3, "z": 0}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 0}}, "num_people": 6, "type": "rectangle",
        "robot_pose": {"position_0": {"x": 0.5, "y": -2, "orientation": 0}, "position_1": {"x": 2.5, "y": -3.5, "orientation": PI_2},
                       "position_2": {"x": 4.5, "y": -2, "orientation": PI}, "position_3": {"x": 2.5, "y": 0.5, "orientation": -PI_2}}},
        {"id": 5, "pose": {"position": {"x": 1.5, "y": 1, "z": 0}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 0}}, "num_people": 4, "type": "square",
        "robot_pose": {"position_0": {"x": 1, "y": 2, "orientation": 0}, "position_1": {"x": 2.5, "y": 0.5, "orientation": PI_2},
                       "position_2": {"x": 4, "y": 2, "orientation": PI}, "position_3": {"x": 2.5, "y": 3.5, "orientation": -PI_2}}}
    ]
}

# Menu (full)
MENU_ALL = {
    "Pizza": 10,
    "Cola": 3,
    "Beer": 5,
}

# Menu (18+)
MENU_N18 = {
    "Beer": 5,
}

class Restaurant():
    """
    Note:
        Mainly methods should be used to update restaurnat attributes
        because multiple robots may connect to the same restaurant and
        updates may be pushed to external databases.
    """

    def __init__(self):
        self.entrance = make_pose(ITEMS["entrance"]["pose"]["position"]["x"], ITEMS["entrance"]["pose"]["position"]["y"] + 1)
        self.kitchen = make_pose(ITEMS["kitchen"]["pose"]["position"]["x"], ITEMS["kitchen"]["pose"]["position"]["y"] + 1)
        self.centre = make_pose(ITEMS["centre"]["pose"]["position"]["x"], ITEMS["centre"]["pose"]["position"]["y"])
        self.tables = [
            Table(id=t["id"], pose=make_pose(t["pose"]["position"]["x"] + (1 if t["type"] == "square" else 1.5), t["pose"]["position"]["y"] + 1),
                  robot_poses=[make_pose(p["x"], p["y"], p["orientation"])
                               for _, p in t["robot_pose"].items()],
                  max_people=t["num_people"])
            for t in ITEMS["tables"]
        ]

        self.order_ids_waiting_food = []
        self.order_ids_waiting_bill = []
        self.order_ids_food_ready = []
        self.order_ids_bill_ready = []
        

        # self.orders = []
        self.order_history = []

        rospy.init_node("restaurant")

    
    def get_menu(self, n18=False):
        """Gets the restaurant menu with food names and prices.

        Args:
            n18 (bool): Whether to _only_ return products marked 18+

        Returns:
            (dict):
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
    

    def get_tables_by_age_group(self, age_group):
        """Gets the list of tables that belong to the given age group.

        Args:
            age_group (range):

        Returns:
            (list(Table)):
        """
        return [table for table in self.tables if table.age_group == age_group]


    def set_occupied_table(self, table_id):
        """Sets the table as occupied (e.g. in database)

        Args:
            table_id (int):
        """
        self.get_table_by_id(table_id).occupied = True


    def get_available_tables(self, num_people=1):
        """Gets the available tables based on the number of people.

        Returns:
            (list(Table)):
        """
        return [table for table in self.tables if not table.occupied and num_people <= table.max_people]


    def set_food_waiting(self, order_id):
        # PUBLISH TO SERVER HERE
        self.order_ids_waiting_food.append(order_id)

    
    def set_bill_waiting(self, order_id):
        # People finished eating and now wait for bill
        # PUBLISH TO SERVER HERE
        self.order_ids_waiting_bill.append(order_id)

    
    def get_food_ready(self):
        """Gets the list of order IDs for which the food is ready.

        Returns:
            (list(int)):
        """
        return self.order_ids_bill_ready

    
    def get_bill_ready(self):
        """Gets the list of order IDs for which the bill is ready.

        Returns:
            (list(int)):
        """
        return self.order_ids_food_ready
    

    def new_customer_exists(self):
        """Checks if there's any new customers at the entrance.

        Returns:
            (bool): Whether any new customer(s) arrived
        """
        return random.choice([True, False])


    def update(self, orders):
        """Updates the state of the restaurant.

        For example, reads a .txt file to see if someone has finished eating.
        Or subscribes to a topic to check for any new updates.
        """
        pass
