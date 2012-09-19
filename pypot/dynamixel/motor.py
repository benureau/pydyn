

class DynamixelMotor(object):
    """ """
    def __init__(self, m_id, model = None, name = '', angle_limits=None, original_pos=None):
        self.id = m_id
        self.model = model
        
        self.name = name
        self.angle_limits = angle_limits
        self.current_position = original_pos
        self.current_speed = None
        self.current_load = None
        self.goal_position = original_pos

        # if flag is True, the controller needs to change other values
        # than the position, speed or load.
        self.flag = False
        
        # Each of those property _p follow the same construction
        #     _p[0] is a flag. If True, the controller needs to update the property on the motor
        #     _p[1] is the desired value
        #     _p[2] is the real value
        # Even if the desired value and the real value don't match, if the flag is not up,
        # the controller will not try to change it. This is used to deal with communication error,
        # and non legal value. All this mechanism is hidden away in properties. 
        self._compliant = [False, None, None]

        self.properties = [self._compliant]

    @property 
    def compliant(self):
        return self._compliant[2]
    
    @compliant.setter
    def compliant(self, val):
        self._compliant[1] = val
        if val != self._compliant[2]:
            self._compliant[0] = True
            self.flag = True
     
    def __repr__(self):
        return 'M{}'.format(self.id)