import debugenv # only to debug local (not installed) pypot version

import pypot
import pypot.dynamixel as dyn

ctrl = dyn.create_controller(verbose = True, motor_range = [0, 10], timeout = 0.02)

print('\ndisplay eeprom ([y]/n) ?'),
r = raw_input()


if r == 'y' or r == '':
    for m in ctrl.motors:
        print('')
        print(m.eeprom_desc())    