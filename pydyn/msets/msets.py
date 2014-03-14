from __future__ import print_function, division

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
        if hasattr(self.__class__, name):
            object.__getattribute__(self, name)
        try:
            if hasattr(self, name):
                object.__getattribute__(self, name)
        except AttributeError:
            pass
        if not any(hasattr(m, name) for m in self.motors):
            raise AttributeError("MotorSet has no attribute '{}'".format(name))
        return tuple(getattr(m, name) for m in self.motors)

    def __setattr__(self, name, values):
        if hasattr(self.__class__, name):
            object.__setattr__(self, name, values)
            return
        if not any(hasattr(m, name) for m in self.motors):
            raise AttributeError("MotorSet has no attribute '{}'".format(name))

        if not hasattr(values, '__iter__'):
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
        if not hasattr(values, '__iter__'):
            values = [values for m in self.motors]
        assert len(values) == len(self.motors), 'Expected at least {} values, got {}'.format(len(self.motors), values)
        object.__setattr__(self, '_zero_pose', values)

    @property
    def pose(self):
        return tuple(m.position - zp for m, zp in zip(self.motors, self.zero_pose))

    @pose.setter
    def pose(self, values):
        if not hasattr(values, '__iter__'):
            values = [values for m in self.motors]
        for m, zp, p in zip(self.motors, self.zero_pose, values):
            m.position = p + zp
