import debugenv # only to debug local (not installed) pypot version

import pypot
import pypot.robot

robot = pypot.robot.Robot(motor_range = [91, 96], timeout = 0.02)
robot.compliant = False
print 'cw', robot.m_by_id[95].cw_angle_limit
print 'ccw', robot.m_by_id[95].ccw_angle_limit
robot.goto((219, 103, 100, 206, 249, 171), max_speed = 100, margin = 1.0)
robot.join()
print 'Finished'
