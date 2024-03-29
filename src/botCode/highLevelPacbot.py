from highLevelVariables import *


class HighLevelPacBot:
    """
        Allows initializing and updating information about PacBot
    """

    def __init__(self):
        self.pos = pacbot_starting_pos
        self.direction = right
        self.prev_direction = right
        # self.respawn()

    def respawn(self):
        self.pos = pacbot_starting_pos
        self.direction = pacbot_starting_dir
        self.prev_direction = pacbot_starting_dir

    def update(self, position):
        self.prev_direction = self.direction
        if position[0] > self.pos[0]:
            self.direction = right
        elif position[0] < self.pos[0]:
            self.direction = left
        elif position[1] > self.pos[1]:
            self.direction = up
        elif position[1] < self.pos[1]:
            self.direction = down
        self.pos = position
