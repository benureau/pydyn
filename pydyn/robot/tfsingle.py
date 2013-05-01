import math

from . import timedf

class Constant(timedf.TimedFunction):
    """Implements a constant timed function of specific duration.

    :param start, stop, duration  in s
    """

    def __init__(self, target, duration = float('inf')):
        self.target = target
        self.duration = duration

    def get_value(self, t):
        return self.target

    def has_finished(self, t):
        return t > self.duration


class AutoGoto(timedf.TimedFunction):
    """Implements a function that stop when arrived within a margin of error of a position

    :param start, stop, duration  in s
    """

    def __init__(self, motor, target, margin = 0.3):
        self.motor = motor
        self.target = target
        self.margin = abs(margin)

    def get_value(self, t):
        return self.target

    def has_finished(self, t):
        if abs(self.motor.position - self.target) <= self.margin:
            return True
        return False

class ProgressGoto(timedf.TimedFunction):
    """Implements a function that stop when not progressing toward the goal anymore.

    :param start, stop, duration  in s
    """

    def __init__(self, motor, target, timeout = 0.2, margin = 0.5):
        self.motor = motor
        self.target = target
        self.closest = None
        self.margin = margin
        self.timeout = timeout

    def get_value(self, t):
        if self.closest is None:
            self.closest = self.motor.position, t
        elif abs(self.motor.position - self.target) < abs(self.closest[0] - self.target):
            self.closest = self.motor.position, t
        return self.target

    def has_finished(self, t):
        if t - self.closest[1] > self.timeout or abs(self.motor.position-self.target) < self.margin:
            return True
        return False


class LinearGoto(timedf.TimedFunction):
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


class Sinus(timedf.TimedFunction):

    def __init__(self, period, amplitude, v_shift = 150.0, phase = 0.0, duration = float('inf')):
        self.omega = 2. * math.pi / period
        self.phi   = phase
        self.a     = amplitude
        self.b     = v_shift
        self.duration = duration

    def get_value(self, t):
        return self.a*math.sin(self.omega * t + self.phi) + self.b

    def has_finished(self, t):
        return t > self.duration

