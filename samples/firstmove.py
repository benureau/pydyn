import debugenv # only to debug local (not installed) pydyn version

import sys, time

import pydyn
import pydyn.dynamixel as dyn

idx = sys.argv[1] if len(sys.argv) == 2 else 0

ctrl = dyn.create_controller(verbose = True, motor_range = [0, 20])

m = ctrl.motors[idx]
#m.id = 15

time.sleep(5.0)

m.compliant = False
m.position  = 100
time.sleep(2.0)

m.position  = 200
time.sleep(2.0)
