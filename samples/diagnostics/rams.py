 import debugenv # only to debug local (not installed) pydyn version

import pydyn
import pydyn.dynamixel as dyn

ctrl = dyn.create_controller(verbose = True, motor_range = [0, 253], start = False, timeout = 0.02)

for m in ctrl.motors:
    print m.ram_desc()