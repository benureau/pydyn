
import motionctrl
import tfsingle
import tftriple

from ..import dynamixel

class SimpleRobot(object):
    """A simple robot with only one controller"""

    def __init__(self, controller=None, motor_range = range(1, 253), timeout = 0.03):
        self._ctrl = controller
        if self._ctrl is None:
            self._ctrl = dynamixel.create_controller(verbose=True,
                                                     motor_range =  motor_range,
                                                     timeout = timeout)

        self.m_by_id = {}
        self.motors  = []
        for m in self._ctrl.motors:
            self.motors.append(m)
            self.m_by_id[m.id] = m
        self.motions = []

    def __repr__(self):
        """Representation of the stem, with motor id and position"""
        return 'Robot({})'.format(', '.join("M%d:%0.f" % (m.id, m.position) for m in self.motors))

    # position

    @property
    def pose(self):
        return tuple(m.position for m in self.motors)

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
                m_i.torque = v_i
        else:
            for m_i in self.motors:
                m_i.torque_limit = v

    # stop, suspend, resume

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
            value = len(motor_ids)*[values]
        return motor_ids, values
    
    def goto(self, motor_id, pos, max_speed = 200.0, duration = float('inf')):
        """Order a straigh motion to goal position with a maximum speed.

        :param pos       in degrees.
        :param max_speed in degree/s. Value over 500 are not recommended.
        :param duration  when to stop the motion
        """
        motor = self.m_by_id[motor_id]
        motor.speed = max_speed
                
        tf = tfsingle.Constant(pos, duration)
        motion = motionctrl.PoseMotionController(motor, tf, freq = 30)
        motion.start()
        self.motions.append(motion)
        
        return motion

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
        
        for motor_id in motor_ids:
            motor = self.m_by_id[motor_id]
            if max_speed is not None:
                motor.speed = max_speed
    
            startpos = motor.current_position
            if duration is None:
                duration = abs(pos - startpos)/motor.moving_speed
    
            tf = tfsingle.LinearGoto(motor.current_position, pos, duration)
            motion = motionctrl.PoseMotionController(motor, tf, freq = 30)
            motion.start()
            motions_created.append(motion)
            self.motions.append(motion)

        return motions_created

    def sinus(self, motor_id, center_pos, amplitude, period = 1.0, duration = float('inf'), max_speed = 200.0):
        """Order a sinus motion around a position.

        :param pos       in degrees.
        :param duration  in s
        :param max_speed in degree/s. Value over 500 are not recommended.
        """

        motor = self.m_by_id[motor_id]
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

        motor = self.m_by_id[motor_id]

        tf = tftriple.DualSinus(motor, period, amplitude, center_pos, duration)
        motion = motionctrl.PoseSpeedTorqueController(motor, tf, freq = 30)
        motion.start()
        self.motions.append(motion)

        return motion

