import debugenv # only to debug local (not installed) pypot version

import pypot
import pypot.dynamixel as dyn

dyn.enable_vrep()
ctrl = dyn.create_controller(ip = '127.0.0.1', motor_range = [0, 10], verbose = True)