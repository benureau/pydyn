"""Connection can be closed and reopen"""

from __future__ import print_function, division
import time

import env
from pydyn import color
from pydyn.refs import protocol as pt
from pydyn.ios.serialio import serialio, serialcom

n = 1000
for latency in [2, 1]:
    sio = serialio.Serial(latency=latency)
    print('I/O open on {}'.format(sio))
    mcom = serialcom.SerialCom(sio)
    mid = [mid for mid in range(0, 253) if mcom.ping(mid)][0]
    mcom.create([mid])

    start = time.time()
    for _ in range(n):
        mcom.get(pt.PRESENT_POS_SPEED_LOAD, [mid])
    print("{} read messages on one motor with latency {} at {}{:.1f}fps{}.".format(
          n, latency, color.red, n/(time.time()-start), color.end))

    mcom.close()
