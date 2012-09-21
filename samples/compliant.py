import os, sys, time
sys.path.append(os.path.join(os.getcwd(), '..'))

import pypot.dynamixel as dyn

ctrl = dyn.create_controller(verbose = True, motor_range = [91, 96])

print 'Making motor compliant'
for m in ctrl.motors:
    m.compliant = True
print 'Done'

time.sleep(0.5)

for m in ctrl.motors:
    print m.id, m.compliant
