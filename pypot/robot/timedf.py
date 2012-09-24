"""
TimedFunctions computes the next values of the motor variables in a motion.
Although they might have access to motor instances, they should only read their
attribute, and not change them in any way.
"""

    # Abstract base classes

class TimedFunction(object):
    """Abstract class for functions called by the motion controller.
    """

    def get_value(self, elapsed_time):
        raise NotImplementedError

    def has_finished(self, t):
        # By default TimedFunction are never finished
        return False


class TimedTripleFunction(object):
    """Abstract class for functions called by the motion controller.
        Return position, speed and max torque at every step
    """

    def get_value(self, elapsed_time):
        """"Compute the value of position, and maximum speed and torque the motor 
            should adopt given the elapsed time
            
            :return  3-length tuple with position (degrees), speed (degree/s), 
                     and torque (N).
                     If one of those value is None, if won't be updated on the motor
        """
        return None, None, None

    def has_finished(self, t):
        # By default TimedFunction are never finished
        return False


    # Compositions
        
class SumTimedFunction(TimedFunction):
    """Sum of two time functions.
    """

    def __init__(self, tf1, tf2):
        self.tf1 = tf1
        self.tf2 = tf2

    def get_value(self, t):
        return self.tf1.get_value(t) + self.tf2.get_value(t)

    def has_finished(self, t):
        return self.tf1.has_finished(t) and self.tf2.has_finished(t)


class SumTripleTimedFunction(TimedTripleFunction):
    """Sum of two triple time functions.
        Position are added. The first function values override the one of the 
        second function, if they are not None.
    """

    def __init__(self, tf1, tf2):
        self.tf1 = tf1
        self.tf2 = tf2

    @staticmethod
    def _nonezero(x):
        if x is None:
            return 0.0
        return x
        
    def get_value(self, t):
        pos1, speed1, torque1 =  self.tf1.get_value(t)
        pos2, speed2, torque2 =  self.tf2.get_value(t)
        pos = self._nonezero(pos1) + self._nonezero(pos2)
        speed = pos1 if pos1 is not None else pos2 
        torque = torque1 if torque1 is not None else torque2 
        return pos, speed, torque

    def has_finished(self, t):
        return self.tf1.has_finished(t) and self.tf2.has_finished(t)
