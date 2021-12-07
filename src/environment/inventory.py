from enum import Enum, auto
from utils.geom import make_pose, PI, PI_2


class OrderStatus(Enum):
    WAITING_SEAT = auto()
    WAITING_FOOD = auto()
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


class TableType(Enum):
    SQUARE = "square"
    RECTANGLE = "rectangle"


class Table():
    def __init__(self, id, pose, max_people, type):
        self.id = id
        self.pose = pose
        self.max_people = max_people
        self.occupied = False
        self.type = TableType(type)
        self.width = self._get_width()
        self.height = self._get_height()
        self.robot_poses = self._init_robot_poses()

    def _get_width(self):
        if self.type == TableType.SQUARE:
            return 2
        elif self.type == TableType.RECTANGLE:
            return 3
        else:
            return 0

    def _get_height(self):
        if self.type == TableType.SQUARE or self.type == TableType.RECTANGLE:
            return 2
        else:
            return 0

    def _init_robot_poses(self):
        # Based on self.pose, self.width and self.length

        safe_dist = 0.6
        height_mid = self.height / 2
        width_mid = self.width / 2

        x = self.pose.pose.position.x
        y = self.pose.pose.position.y

        poses = []
        poses.append(make_pose(x - safe_dist,
                               y + height_mid, 0))
        poses.append(make_pose(x + width_mid,
                               y - safe_dist, PI_2))
        poses.append(make_pose(x + self.width + safe_dist,
                               y + height_mid, PI))
        poses.append(make_pose(x + width_mid,
                               y + self.height + safe_dist, -PI_2))

        return poses
