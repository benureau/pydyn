import debugenv # only to debug local (not installed) pypot version

import time

import pypot
import pypot.robot

robot = pypot.robot.Robot(motor_range = [91, 96], timeout = 0.02)

print 'Press Enter when ready to capture position... ',
raw_input()
print robot.pose