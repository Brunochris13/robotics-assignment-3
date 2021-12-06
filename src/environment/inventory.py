from enum import Enum, auto


class OrderStatus(Enum):
    WAITING_SEAT = auto()
    WAITING_FOOD = auto()
    PROCESS_FOOD = auto()
    WAITING_BILL = auto()
    FINISHED = auto()
    CANCELED = auto()


class Order():
    next_id = 0

    def __init__(self, table=None, people=None):
        self.id = Order.next_id
        self.status = OrderStatus.WAITING_SEAT
        self.table = table
        self.people = people
        self.contents = []

        Order.next_id += 1


class Table():
    def __init__(self, id, pose, robot_poses, max_people, width=None, length=None):
        self.id = id
        self.pose = pose
        self.robot_poses = robot_poses # self._init_robot_poses()
        self.max_people = max_people
        self.occupied = False
    
    def _init_robot_poses(self):
        # Based on self.pose, self.width and self.length
        pass
