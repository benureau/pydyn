import debugenv # only to debug local (not installed) pypot version

import sys, time
import pypot.dynamixel as dyn

ctrl = dyn.create_controller(verbose = True, motor_range = [91, 96])

print '\nMaking motor compliant... ',
sys.stdout.flush()

for m in ctrl.motors:
    m.compliant = True

time.sleep(0.5)
print 'done'

for m in ctrl.motors:
    print "{} compliant : {}".format(m, m.compliant)
