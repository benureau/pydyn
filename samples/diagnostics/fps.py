import debugenv # only to debug local (not installed) pypot version

import sys, time
import pypot.dynamixel as dyn

ctrl = dyn.create_controller(verbose = True, motor_range = [91, 96])

time.sleep(3.0)
print 'FPS (last 2s): {}'.format(ctrl.fps)
