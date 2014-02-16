import env

import pydyn.io.ftdiserial as ftdiserial

s = ftdiserial.Serial(serial_id='A4008aCD')
assert s._ftdi_ctrl
s2 = ftdiserial.Serial(serial_id='A8006BPa')
assert s2._ftdi_ctrl

s.close()
s2.close()
