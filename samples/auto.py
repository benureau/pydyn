import os, sys
sys.path.insert(0, (os.path.join(os.getcwd(), '..')))

import pydyn.dynamixel as dyn

ctrl = dyn.create_controller(verbose = True, motor_range=(1, 50), timeout=20, enable_pyftdi=True)
print ctrl.motors[0].mmem._memory_data
