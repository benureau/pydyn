from __future__ import print_function, division

import collections

from ..dynamixel import hub

def _distribute(functions):
    pass

class MotorSet(object):

    def __init__(self, motors=None, n=1, **kwargs):
        """Instanciate a motor set.

        :param motors:  list of Motor instances. If None, will try to detect
                        devices, instanciate controllers and load motors.
        :param n:       the number of controller to instanciate. Has no effect
                        if ``motors`` is not None.
        """
        object.__setattr__(self, 'motors', motors)
        if self.motors is None:
            dyn_uid  = hub.connect(**kwargs) # TODO treat n
            object.__setattr__(self, 'motors', tuple(hub.motors(dyn_uid)))
        object.__setattr__(self, '_zero_pose', tuple(0.0 for m in self.motors))

    def __getattr__(self, name):
        if hasattr(self.__class__, name) or name in self.__dict__:
            object.__getattribute__(self, name)
        if len(self.motors) == 0:
            raise AttributeError("No motor in motor set")
        if not any(hasattr(m, name) for m in self.motors):
            raise AttributeError("MotorSet has no attribute '{}'".format(name))
        return tuple(getattr(m, name) for m in self.motors)

    def __setattr__(self, name, values):
        if hasattr(self.__class__, name) or name in self.__dict__:
            object.__setattr__(self, name, values)
            return
        if len(self.motors) == 0:
            try:

                if len(values) == 0:
                    return
            except TypeError:
                raise AttributeError("No motor in motor set")
        if not any(hasattr(m, name) for m in self.motors):
            raise AttributeError("MotorSet has no attribute '{}'".format(name))

        if not isinstance(values, collections.Iterable):
            values = [values for m in self.motors]
        failcount = 0
        for m, val in zip(self.motors, values):
            try:
                setattr(m, name, val)
            except AttributeError:
                failcount += 1

        if failcount == len(self.motors):
            raise AttributeError("MotorSet can't set attribute '{}'".format(name))

    @property
    def zero_pose(self):
        return self._zero_pose

    @zero_pose.setter
    def zero_pose(self, values):
        if not isinstance(values, collections.Iterable):
            values = [values for m in self.motors]
        assert len(values) == len(self.motors), 'Expected at least {} values, got {}'.format(len(self.motors), values)
        object.__setattr__(self, '_zero_pose', values)

    @property
    def pose(self):
        return tuple(m.position - zp for m, zp in zip(self.motors, self.zero_pose))

    @pose.setter
    def pose(self, values):
        if not isinstance(values, collections.Iterable):
            values = [values for m in self.motors]
        for m, zp, p in zip(self.motors, self.zero_pose, values):
            m.position = p + zp

    def close_all(self):
        hub.close_all()

    def __dir__(self):
        motor_dir = []
        for motor in self.motors:
            motor_dir += dir(motor)
        return sorted(set(dir(type(self)) +
                          self.__dict__.keys() + motor_dir))
