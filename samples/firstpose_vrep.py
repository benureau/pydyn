import debugenv # only to debug local (not installed) pydyn version

import time

import pydyn
import pydyn.robot
pydyn.enable_vrep()

robot = pydyn.robot.Robot(motor_range = [1, 6])

robot.compliant = False
robot.speed = 30
robot.pose = 50.0

print robot
#robot.join()
start = time.time()
for i in xrange(50):
    print "{:5.2f}: ({})".format(i/10.0, ', '.join('{:5.2f}'.format(s_i) for s_i in robot.pose))
    time.sleep(0.1)

print("\ntook {:.3f} s".format(time.time()-start))