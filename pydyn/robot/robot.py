
import motionctrl
import tfsingle
import tftriple

from ..import dynamixel

class Robot(object):
    """A simple robot with only one controller"""

    def __init__(self, controller=None, motor_range = range(1, 253), timeout = 0.03, start = True, full_ram = False):
        self._ctrl = controller
        if self._ctrl is None:
            self._ctrl = dynamixel.create_controller(verbose=True,
                                                     motor_range =  motor_range,
                                                     timeout = timeout,
                                                     start = start,
                                                     full_ram = full_ram)


        self.id2m = {}
        self.motors  = []
        for m in self._ctrl.motors:
            self.motors.append(m)
            self.id2m[m.id] = m
        self.motors.sort()
        self.motions = []

    def __repr__(self):
        """Representation of the stem, with motor id and position"""
        return 'Robot({})'.format(', '.join("M%d:%0.f" % (m.id, m.position) for m in self.motors))

    def get_sim(self):
        """If vrep_mode is enable, return the simulation instance.
        Else, raise a AttributeError exception
        """
        return self._ctrl.io.sim

    # position

    @property
    def pose(self):
        return tuple(m.position for m in self.motors)

    @property
    def goal_pose(self):
        return tuple(m.goal_position for m in self.motors)

    @pose.setter
    def pose(self, p):
        """Set target position.

        If only one value is provided, set all motor to the same value.
        Else, expect an iterable of length superior to the number of motors

        note:: Be careful at specifying a reasonable motor speed
        """
        if hasattr(p, '__iter__'):
            assert len(p) >= len(self.motors)
            for m_i, p_i in zip(self.motors, p):
                m_i.position = p_i
        else:
            for m_i in self.motors:
                m_i.position = p

    position = pose

    # speed

    @property
    def speed(self):
        return tuple(m.speed for m in self.motors)

    @speed.setter
    def speed(self, p):
        """Set target position.

        If only one value is provided, set all motor to the same value.
        Else, expect an iterable of length superior to the number of motors
        """
        if hasattr(p, '__iter__'):
            assert len(p) >= len(self.motors)
            for m_i, p_i in zip(self.motors, p):
                m_i.speed = p_i
        else:
            for m_i in self.motors:
                m_i.speed = p

    # compliance

    @property
    def compliant(self):
        return tuple(m.compliant for m in self.motors)

    @compliant.setter
    def compliant(self, v):
        """Enable or disable compliance.

        If only one value is provided, set all motor to the same value.
        Else, expect an iterable of length superior to the number of motors
        """
        if hasattr(v, '__iter__'):
            assert len(v) >= len(self.motors)
            for m_i, v_i in zip(self.motors, v):
                m_i.compliant = v_i
        else:
            for m_i in self.motors:
                m_i.compliant = v

    # torque

    @property
    def torque(self):
        return tuple(m.torque_limit for m in self.motors)

    @compliant.setter
    def torque(self, v):
        """Enable or disable compliance.

        If only one value is provided, set all motor to the same value.
        Else, expect an iterable of length superior to the number of motors
        """
        if hasattr(v, '__iter__'):
            assert len(v) >= len(self.motors)
            for m_i, v_i in zip(self.motors, v):
                m_i.torque_limit = v_i
        else:
            for m_i in self.motors:
                m_i.torque_limit = v

    # stop, suspend, resume

    def close(self,  immediately = False):
        """Close the robot : allow two more run of the controller"""
        self._ctrl.close(immediately = immediately)

    def stop(self):
        """Stop all motions"""
        for motion in self.motions:
            if motion.is_alive():
                motion.stop()
        self.motions = []

    def suspend(self, still = True):
        """Suspend all motions

        :param still  if True, the robot motor stop immediately.
        """
        for motion in self.motions:
            if motion.is_alive():
                motion.suspend()
        if still:
            for m in self.motors:
                m.stop()
        self._clean_up()

    def resume(self):
        """Resume all motions"""
        for motion in self.motions:
            if motion.is_alive():
                motion.resume()

    def join(self):
        """Wait for all motion to finish"""
        self._clean_up()
        for motion in self.motions:
            if motion.is_alive():
                motion.join()

    def _clean_up(self):
        """Clean-up dead motions"""
        self.motions = [motion for motion in self.motions if motion.is_alive()]

    # motion

    @staticmethod
    def _prepare(motor_ids, values):
        """Translate motor_ids and value into two lists"""
        if not hasattr(values, '__iter__'):
            if not hasattr(motor_ids, '__iter__'):
                return [motor_ids], [values]
            else:
                return motor_ids, len(motor_ids)*[values]
        return motor_ids, values

    def goto(self, pos, motor_ids = None, margin = 0.2, max_speed = 200.0):
        """Order a straigh motion to goal position with a maximum speed.

        :param pos       in degrees.
        :param margin    in degrees. Once the motors position is within the margin
                         of the target position, the motion will stop.
        :param max_speed in degree/s. Value over 500 are not recommended.
        """
        motor_ids = motor_ids or [m.id for m in self.motors]
        motor_ids, pos = self._prepare(motor_ids, pos)
        motions_created = []

        for pos_i, motor_id in zip(pos, motor_ids):
            motor = self.id2m[motor_id]
            motor.speed = max_speed

#            tf = tfsingle.AutoGoto(motor, pos_i, margin)
            tf = tfsingle.ProgressGoto(motor, pos_i)
            motion = motionctrl.PoseMotionController(motor, tf, freq = 30)
            motion.start()
            self.motions.append(motion)
            motions_created.append(motion)

        return motions_created

    def constantspeed(self, speeds, motor_ids = None, duration = float('inf')):
        motor_ids = motor_ids or [m.id for m in self.motors]
        motor_ids, speeds = self._prepare(motor_ids, speeds)
        motions_created = []

        for speed_i, motor_id in zip(speeds, motor_ids):
            motor = self.id2m[motor_id]

            tf = tfsingle.Constant(speed_i, duration)
            motion = motionctrl.SpeedController(motor, tf, freq = 30)
            motion.start()
            self.motions.append(motion)
            motions_created.append(motion)

        return motions_created


    def linear(self, pos, motor_ids = None, duration = None, max_speed = None):
        """Order a linear motion for the position.

        :param pos        in degrees. If motor_ids is an iterable, an iterable is also accepted
        :param motor_ids  motor ids. If None, considers all motors of the robot.
        :param duration   in s. If None, it is automatically computed
        :param max_speed  in degree/s. Value over 500 are not recommended. If None, no effect.

        :return list of motions created if motor_ids is iterable, a single motion otherwise.
        """
        motor_ids = motor_ids or [m.id for m in self.motors]
        motor_ids, pos = self._prepare(motor_ids, pos)
        motions_created = []

        for pos_i, motor_id in zip(pos, motor_ids):
            motor = self.id2m[motor_id]
            if max_speed is not None:
                motor.speed = max_speed

            startpos = motor.current_position
            if duration is None:
                duration = abs(pos_i - startpos)/motor.moving_speed

            tf = tfsingle.LinearGoto(motor.current_position, pos_i, duration)
            motion = motionctrl.PoseMotionController(motor, tf, freq = 30)
            motion.start()
            self.motions.append(motion)
            motions_created.append(motion)

        return motions_created

    def sinus(self, motor_id, center_pos, amplitude, period = 1.0, duration = float('inf'), max_speed = 200.0):
        """Order a sinus motion around a position.

        :param pos       in degrees.
        :param duration  in s
        :param max_speed in degree/s. Value over 500 are not recommended.
        """

        motor = self.id2m[motor_id]
        motor.speed = max_speed

        tf = tfsingle.Sinus(period, amplitude, center_pos, duration)
        motion = motionctrl.PoseMotionController(motor, tf, freq = 30)
        motion.start()
        self.motions.append(motion)

        return motion

    def sinus2(self, motor_id, center_pos, amplitude, period = 1.0, duration = float('inf')):
        """Order a sinus motion around a position. Speed is dynamically adjusted.

        :param pos       in degrees.
        :param duration  in s
        :param max_speed in degree/s. Value over 500 are not recommended.
        """

        motor = self.id2m[motor_id]

        tf = tftriple.DualSinus(motor, period, amplitude, center_pos, duration)
        motion = motionctrl.PoseSpeedTorqueController(motor, tf, freq = 30)
        motion.start()
        self.motions.append(motion)

        return motion

