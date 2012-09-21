import math

class TimedFunction:
    """Abstract class for functions called by the motion controller.
    """

    def get_value(self, elapsed_time):
        raise NotImplementedError

    def has_finished(self, t):
        # By default TimedFunction are never finished
        return False

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