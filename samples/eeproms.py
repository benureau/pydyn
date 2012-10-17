import debugenv # only to debug local (not installed) pypot version

import pypot
import pypot.dynamixel as dyn

ctrl = dyn.create_controller(verbose = True, motor_range = [13, 17], start = False, timeout = 1)

for m in ctrl.motors:
    print m.eeprom_repr()