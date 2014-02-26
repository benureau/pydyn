import time

from .. import dynamixel

class MotorSet(object):

    def __init__(self, **kwargs):
        self.dyn  = dynamixel.create_controller(**kwargs)
        self.motors = self.dyn.motors
        self._angle_ranges = None
        self.zero_pose = [0.0]*len(self.motors)

    def close(self):
        pass

    @property
    def angle_ranges(self):
        return self._angle_ranges

    @angle_ranges.setter
    def angle_ranges(self, rbounds):
        assert len(rbounds) == len(self.motors)
        self._angle_ranges = tuple((min(zp+1, lr), min(299-zp, hr)) for (lr, hr), zp in zip(rbounds, self.zero_pose))

    def _clip(self, pose):
        if self._angle_ranges is not None:
            return tuple([max(min(p, zp+hr), zp-lr) for p, zp, (lr, hr) in zip(pose, self.zero_pose, self.angle_ranges)])
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
        return tuple([p - sp for sp, p in zip(self.pose, pose)])

    @property
    def pose(self):
        return tuple([m.position - zp for m, zp in zip(self.motors, self.zero_pose)])

    @pose.setter
    def pose(self, _pose):
        #print("bla" ,tuple(_pose) + self.zero_pose)
        #print("blu" ,self._clip(tuple(_pose) + self.zero_pose))
        for m, p in zip(self.motors, self._clip(tuple(_pose) + self.zero_pose)):
            m.position = p

    @property
    def max_speed(self):
        return tuple([m.speed for m in self.motors])

    @max_speed.setter
    def max_speed(self, _speed):
        for m in self.motors:
            m.speed = _speed

    @property
    def compliant(self):
        return [m.compliant for m in self.motors]

    @compliant.setter
    def compliant(self, val):
        for m in self.motors:
            m.compliant = val

    @property
    def torque_limit(self):
        return [m.torque_limit for m in self.motors]

    @torque_limit.setter
    def torque_limit(self, val):
        for m in self.motors:
            m.torque_limit = val

    @property
    def max_torque(self):
        return [m.max_torque for m in self.motors]

    @max_torque.setter
    def max_torque(self, val):
        for m in self.motors:
            m.max_torque = val

    @property
    def angle_limits(self):
        return [m.max_torque for m in self.motors]

    @max_torque.setter
    def angle_limits(self, val):
        assert len(self.motors) == len(val)
        for m, v_i in zip(self.motors, val):
            m.angle_limits = v_i


    @property
    def zero_pose(self):
        return self._zero_pose

    @zero_pose.setter
    def zero_pose(self, val):
        assert all(p>=0 for p in val)
        self._zero_pose = tuple(val)

