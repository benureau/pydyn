
import motionctrl
import timedf

from ..import dynamixel

class SimpleRobot(object):
    """A simple robot with only one controller"""

    def __init__(self, controller=None, motor_range = range(1, 253)):
        self.ctrl = controller
        if self.ctrl is None:
            self.ctrl = dynamixel.create_controller(verbose=True, motor_range =  motor_range)
        
        self.motors = {}
        for m in self.ctrl.motors:
            self.motors[m.id] = m
        self.motions = []
        
    def goto(self, motor_id, pos, max_speed = 200.0):
        """Order a straigh motion to goal position with a maximum speed.
        
        :note This is not handled through a motion controller, but as a 
              one-shot manipulation of the motor variables
        
        :param pos       in degrees.
        :param max_speed in degree/s. Value over 500 are not recommended.
        """
        print pos, max_speed
        motor = self.ctrl.motormap[motor_id]
        motor.goal_position = pos
        motor.speed = max_speed/6
        
    def linear(self, motor_id, pos, duration = None, max_speed = 200.0):
        """Order a linear motion for the position.
        
        :param pos       in degrees.
        :param duration  in s
        :param max_speed in degree/s. Value over 500 are not recommended.
        """
        
        motor = self.ctrl.motormap[motor_id]
        motor.speed = max_speed/6
        
        startpos = motor.current_position
        if duration is None:
            duration = abs(pos - startpos)/max_speed
            
        tf = timedf.LinearGoto(motor.current_position, pos, duration)
        motion = motionctrl.PoseMotionController(motor, tf, freq = 30)
        motion.start()
        self.motions.append(motion)
        
        return motion

    def sinus(self, motor_id, center_pos, amplitude, period = 1.0, duration = float('inf'), max_speed = 200.0):
        """Order a linear motion for the position.
        
        :param pos       in degrees.
        :param duration  in s
        :param max_speed in degree/s. Value over 500 are not recommended.
        """
        
        motor = self.ctrl.motormap[motor_id]
        motor.speed = max_speed/6
        
        tf = timedf.Sinus(period, amplitude, center_pos, duration)
        motion = motionctrl.PoseMotionController(motor, tf, freq = 30)
        motion.start()
        self.motions.append(motion)
        
        return motion
