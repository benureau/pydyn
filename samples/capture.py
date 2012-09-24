import os, sys, time
sys.path.insert(0, (os.path.join(os.getcwd(), '..')))

import pypot
import pypot.robot

robot = pypot.robot.SimpleRobot(motor_range = [91, 96], timeout = 0.02)

print 'Press Enter when ready to capture position... ',
raw_input()
print robot.pose