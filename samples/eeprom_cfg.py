import env # only to debug local (not installed) pydyn version

import time
import pydyn
import pydyn.dynamixel as dyn

ctrl = dyn.create_controller(verbose = True, motor_range = [0, 253], start = True)

for m in ctrl.motors:
    m.status_return_level_bytes = 1
    m.return_delay_time_bytes = 0

time.sleep(1.0)

for m in ctrl.motors:
    print m.eeprom_desc()