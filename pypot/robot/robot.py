
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


    # position

    @property
    def pos(self):
        return tuple(m.position for m in self.motors)

    @pos.setter
    def pos(self, p):
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
        
    def suspend(self):
        """Suspend all motions"""
        for motion in self.motions:
            if motion.is_alive():            
                motion.suspend()
        self._clean_up()

    def resume(self):
        """Resume all motions"""
        for motion in self.motions:
            if motion.is_alive():            
                motion.resume()

    def _clean_up(self):
        """Clean-up dead motions"""
        self.motions = [motion for motion in self.motions if motion.is_alive()]

    # motion

    def goto(self, motor_id, pos, max_speed = 200.0):
        """Order a straigh motion to goal position with a maximum speed.

        :note This is not handled through a motion controller, but as a
              one-shot manipulation of the motor variables

        :param pos       in degrees.
        :param max_speed in degree/s. Value over 500 are not recommended.
        """
        print pos, max_speed
        motor = self.m_by_id[motor_id]
        motor.goal_position = pos
        motor.speed = max_speed

    def linear(self, motor_id, pos, duration = None, max_speed = 200.0):
        """Order a linear motion for the position.

        :param pos       in degrees.
        :param duration  in s
        :param max_speed in degree/s. Value over 500 are not recommended.
        """

        motor = self.m_by_id[motor_id]
        motor.speed = max_speed

        startpos = motor.current_position
        if duration is None:
            duration = abs(pos - startpos)/max_speed

        tf = tfsingle.LinearGoto(motor.current_position, pos, duration)
        motion = motionctrl.PoseMotionController(motor, tf, freq = 30)
        motion.start()
        self.motions.append(motion)

        return motion

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

