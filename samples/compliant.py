import os, sys
sys.path.append(os.path.join(os.getcwd(), '..'))

import pypot.dynamixel as dyn

ctrl = dyn.create_controller(verbose = True)

print 'Making motor compliant'
for m in ctrl.motors:
    m.compliant = True
print 'Done'

time.sleep(0.2)