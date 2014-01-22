import numpy as np

from .. import dynamixel

class MotorSet(object):

    def __init__(self, motor_range=(0, 254), verbose=False):
        self.dyn  = dynamixel.create_controller(motor_range = motor_range,
                                                verbose = verbose)
        self.motors = self.dyn.motors
        self._range_bounds = None

    def close(self):
        pass

    @property
    def range_bounds(self):
        return self._range_bounds

    @range_bounds.setter
    def range_bounds(self, rbounds):
        assert len(rbounds) == len(self.motors)
        self._range_bounds = tuple((max(52, lb), min(247, hb)) for lb, hb in rbounds)

    def _clip(self, pose):
        if self._range_bounds is not None:
            return np.array([max(min(p, hb), lb) for p, (lb, hb) in zip(pose, self.range_bounds)])
        else:
            return pose

    @property
    def fps(self):
        return self.dyn.fps()

    def go_to(self, pose, margin = 1.5, timeout = 10.0):
        self.pose = pose
        start = time.time()
        while max(abs(sp - p) for sp, p in zip(self.pose, pose)) > margin and time.time()-start < timeout:
            time.sleep(0.01)
        return max(abs(sp - p) for sp, p in zip(self.pose, pose))

    @property
    def pose(self):
        return np.array([m.position for m in self.motors])

    @pose.setter
    def pose(self, _pose):
        for m, p in zip(self.motors, self._clip(_pose)):
            m.position = p

    @property
    def max_speed(self):
        return np.array([m.speed for m in self.motors])

    @max_speed.setter
    def max_speed(self, _speed):
        for m in self.motors:
            m.speed = _speed

    @property
    def compliant(self):
        return any(m.compliant for m in self.motors)

    @compliant.setter
    def compliant(self, val):
        for m in self.motors:
            m.compliant = val

    @compliant.setter
    def max_torque(self, val):
        for m in self.motors:
            m.max_torque = val

