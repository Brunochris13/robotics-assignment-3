import math
from utils.geom import is_near

class Table():
    def __init__(self, id, pos, max_people, entrance_pos=None, centre_pos=None):
        self.id = id
        self.pos = pos
        self.max_people = max_people
        self.age_group = self.assign_age_group(entrance_pos, centre_pos)
        self.occupied = False
    

    def describe(self, show_id=True, show_pose=True, show_max_people=False):
        description = ""

        if show_id:
            description += f"{self.id} "
        
        if show_pose:
            description += f"({self.pos.position.x}, {self.pos.position.y}) "
        
        if show_max_people:
            description += f"[MAX: {self.max_people}]"

        return description

    def assign_age_group(self, entrance_pos, centre_pos):
        MAX_AGE = 100
        THRESHOLD = 7

        if entrance_pos is None or centre_pos is None:
            return range(MAX_AGE)
        
        radius = None # TODO e.g., sqrt(x^2 +y^2)

        if not is_near(self.pos, centre_pos, radius):
            raise AttributeError("Table is outside restaurant boundaries!")
        
        table_distance_from_entrance = None # TODO
        max_distance_from_entrance = None # TODO

        proportion = 1 - table_distance_from_entrance / max_distance_from_entrance
        mean_age_group = proportion * MAX_AGE

        min_age = math.min(0, math.floor(mean_age_group - THRESHOLD))
        max_age = math.max(100, math.ceil(mean_age_group + THRESHOLD))

        return range(min_age, max_age)
        

        
