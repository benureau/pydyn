import time
import os, sys
sys.path.insert(0, (os.path.join(os.getcwd(), '..')))

import pypot
import pypot.robot

robot = pypot.robot.SimpleRobot(motor_range = [91, 96], timeout = 0.02)

robot.compliant = False
robot.speed = 5
robot.pos = 150.0

#robot.join()
for i in xrange(50):
    print "{:5.2f}: ({})".format(i/10.0, ', '.join('{:5.2f}'.format(s_i) for s_i in robot.speed))
    time.sleep(0.1)
