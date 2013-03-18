import debugenv # only to debug local (not installed) pydyn version

import sys, time
import pydyn.dynamixel as dyn

ctrl = dyn.create_controller(verbose = True, motor_range = [11, 16], full_ram = True)

time.sleep(3.0)
print 'FPS (last 2s): {}'.format(ctrl.fps)
