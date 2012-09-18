

class DynamixelMotor(object):
    """ """
    def __init__(self, id, name, type, angle_limits=None, original_pos=None):
        self.id = id
        self.name = name
        self.type = type
        self.angle_limits = angle_limits
        self.current_position = original_pos
        self.goal_position = original_pos


