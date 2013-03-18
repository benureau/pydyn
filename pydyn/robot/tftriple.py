import math

import timedf

class DualSinus(timedf.TimedTripleFunction):
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
        speed = abs(120*math.sin(self.omega * t + self.phi))
        self._lastt = t
        return position, speed, None

    def has_finished(self, t):
        return t > self.duration

class Still(timedf.TimedTripleFunction):
    """Stabilize the motor"""
    pass