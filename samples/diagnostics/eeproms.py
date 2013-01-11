import debugenv # only to debug local (not installed) pypot version

import pypot
import pypot.dynamixel as dyn

ctrl = dyn.create_controller(verbose = True, motor_range = [0, 253], start = False)

for m in ctrl.motors:
    print m.eeprom_desc()