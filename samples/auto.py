import os, sys
sys.path.append(os.path.join(os.getcwd(), '..'))

import pypot.dynamixel as dyn

ctrl = dyn.create_controller(verbose = True)
