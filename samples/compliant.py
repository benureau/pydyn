import debugenv # only to debug local (not installed) pydyn version

import sys, time
import pydyn.dynamixel as dyn

if len(sys.argv) == 2:
    min_id, max_id = int(sys.argv[1]), int(sys.argv[1])
else:
    min_id, max_id = 0, 253

ctrl = dyn.create_controller(verbose = True, motor_range = [min_id, max_id])

print '\nMaking motor compliant... ',
sys.stdout.flush()

for m in ctrl.motors:
    #m.request_write('compliant', True)
    m.compliant = True

ctrl.wait(2)
print 'done'

for m in ctrl.motors:
    print "{} compliant : {}".format(m, m.compliant)

