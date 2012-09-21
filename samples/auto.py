import os, sys
sys.path.insert(0, (os.path.join(os.getcwd(), '..')))

import pypot.dynamixel as dyn

ctrl = dyn.create_controller(verbose = True)
