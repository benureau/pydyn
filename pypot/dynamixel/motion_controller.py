import threading
import math


class MotionController(threading.Thread):
    pass


class TimedFunction:
    """Abstract class for functions called by the motion controller.
    """

    def get_value(self, elapsed_time):
        raise NotImplementedError

    def has_finished(self, t):
        # By default TimedFunction are never finished
        return False


def Sinus(TimedFunction):

    def __init__(self, freq, phase):
        self.omega = 2. * math.pi * freq
        self.phi = phase

    def get_value(self, t):
        return math.sinus(self.omega * t + self.phi)


def LinearGoto(TimedFunction):
    """Implements a goal moving linearly from a given start to a given stop
    position.

    Params
    ------
    start, stop
    duration: in s
    """

    def __init__(self, start, stop, duration):
        self.start = start
        self.length = stop - start
        self.duration = float(duration)

    def get_value(self, t):
        return self.start + min(max(0, t / self.duration), 1) * self.length

    def has_finished(self, t):
        return t > self.duration
