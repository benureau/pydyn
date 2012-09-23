import math

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


    # Concrete instances

class Sinus(TimedFunction):

    def __init__(self, period, amplitude, v_shift = 150.0, phase = 0.0, duration = float('inf')):
        self.omega = 2. * math.pi / period
        self.phi   = phase
        self.a     = amplitude
        self.b     = v_shift
        self.duration = duration

    def get_value(self, t):
        return min(299, max(1, self.a*math.sin(self.omega * t + self.phi) + self.b))

    def has_finished(self, t):
        return t > self.duration


class DualSinus(TimedTripleFunction):
    """A sinus function that also controls max speed"""

    def __init__(self, motor, period, amplitude, v_shift = 150.0, phase = 0.0, duration = float('inf')):
        self.motor = motor
        self.omega = 2. * math.pi / period
        self.phi   = phase
        self.a     = amplitude
        self.b     = v_shift
        self.duration = duration
        self._lastt   = 0.0
        self._lastpos = 0.0

    def get_value(self, t):
        position = min(299, max(1, self.a*math.sin(self.omega * t + self.phi) + self.b))
        #print "%.0f" % (abs(position-self.motor.position)/(t - self._lastt)/20,)
        speed = abs(20*math.sin(self.omega * t + self.phi))
        self._lastt = t
        return position, speed, None

    def has_finished(self, t):
        return t > self.duration


class LinearGoto(TimedFunction):
    """Implements a goal moving linearly from a given start to a given stop
    position.

    :param start, stop, duration  in s
    """

    def __init__(self, start, stop, duration):
        self.start = start
        self.length = stop - start
        self.duration = float(duration)

    def get_value(self, t):
        return self.start + min(max(0, t / self.duration), 1) * self.length

    def has_finished(self, t):
        return t > self.duration
        
class SumTimeFunction(TimedFunction):
    """Sum of two time functions.
    """

    def __init__(self, tf1, tf2):
        self.tf1 = tf1
        self.tf2 = tf2

    def get_value(self, t):
        return self.tf1.get_value(t) + self.tf2.get_value(t)

    def has_finished(self, t):
        return self.tf1.has_finished(t) and self.tf2.has_finished(t)
        
    