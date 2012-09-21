
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
        
    def goto(self, motor_id, pos, duration = None, max_speed = 90.0):
        """Order a linear motion for the position.
        
        :param pos       in degrees.
        :param duration  in s
        :param max_speed in degree/s. Value over 180 are not recommended.
        """
        
        motor = self.ctrl.motormap[motor_id]
        startpos = motor.current_position
        if duration is None:
            duration = abs(pos - startpos)/90.0
            
        tf = timedf.LinearGoto(motor.current_position, pos, duration)
        motion = motionctrl.PoseMotionController(motor, tf, freq = 30)
        motion.start()
        self.motions.append(motion)
        
        return motion
        
    
    def motor_pos(self, motor_id):
        motor = self.ctrl.motormap[motor_id]
        return motor.current_position