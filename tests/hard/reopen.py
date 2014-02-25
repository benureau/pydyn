"""Connection can be closed and reopen"""

from __future__ import print_function, division
import time

import env
from pydyn.refs import protocol as pt
from pydyn.ios.serialio import serialio, serialcom

for _ in range(2):
    sio = serialio.Serial()
    print('I/O open on {}'.format(sio))
    mcom = serialcom.SerialCom(sio)
    mids = [mid for mid in range(0, 253) if mcom.ping(mid)]

    assert len(mids) > 0, "no motor founds, test can't be run"
    mcom.create(mids[:1])
    mcom.get(pt.PRESENT_POS_SPEED_LOAD, mids[:1])

    mcom.close()

print('all done !')