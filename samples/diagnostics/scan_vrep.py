import debugenv # only to debug local (not installed) pydyn version

import pydyn
import pydyn.dynamixel as dyn

pydyn.enable_vrep()
ctrl = dyn.create_controller(ip = '127.0.0.1', motor_range = [0, 10], verbose = True)