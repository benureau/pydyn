import debugenv # only to debug local (not installed) pydyn version

import time

import pydyn
import pydyn.robot

robot = pydyn.robot.Robot(motor_range = [91, 96], timeout = 0.02)

print 'Press Enter when ready to capture position... ',
raw_input()
print robot.pose