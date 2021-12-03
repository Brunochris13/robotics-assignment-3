from navigation.provide_goal_pose import pub_goal_pose


class Moving():
    def __init__(self):
        pass

    def move_to(self, target_pos):
        pub_goal_pose(target_pos.position.x, target_pos.position.y, 0)

    def turn_to(self, target_pos):
        pass