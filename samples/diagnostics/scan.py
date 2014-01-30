import debugenv # only to debug local (not installed) pydyn version

import sys

import pydyn
import pydyn.dynamixel as dyn

mode = sys.argv[1] if len(sys.argv) == 2 else None


if mode == 'deep':
    for bps in [1000000, 500000, 400000, 250000, 200000, 115200, 57600, 19200, 9600]:
        ctrl = dyn.create_controller(verbose=True, motor_range=[0, 253],
                                     timeout=50, baudrate=bps, start=False)
        ctrl.close()
else:
    ctrl = dyn.create_controller(verbose=True, motor_range=[0, 253], timeout=50, start=False)
    ctrl.close()