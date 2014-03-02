from __future__ import print_function, division
import time

import env
from pydyn.refs import protocol as pt
from pydyn.ios.serialio import serialio, serialcom
from pydyn import color


sio = serialio.Serial(device_type='USB2Serial', latency=1, timeout=0)
print('I/O open on {}{}{}'.format(color.cyan, sio, color.end))
mcom = serialcom.SerialCom(sio)
mid = [mid for mid in range(0, 253) if mcom.ping(mid)][0]
mcom.create([mid])
n = 200

def blackout_duration(writes):
    global mid
    start = time.time()
    for i in range(n):
        mcom.get(pt.PRESENT_POS_SPEED_LOAD, [mid])

    for control, values, _ in writes:
        if control == pt.ID:
            mcom.change_id(mid, values[0])
            mid = values[0]
        else:
            mcom.set(control, [mid], (values,))
            #mcom.set(control, [mid], (values,))
            #mcom.set(control, [mid], (values,))

    start = time.time()
    no_error = 0
    timeouts  = 0
    comerrors = 0
    while no_error < 50:
        try:
            mcom.get(pt.PRESENT_POS_SPEED_LOAD, [mid])
            if no_error == 0:
                end = time.time()
            no_error += 1
        except mcom.TimeoutError:
            timeouts += 1
            no_error = 0
        except mcom.CommunicationError as e:
            #print(list(e.status_packet))
            comerrors += 1
            no_error = 0

    # should pass without error
    for i in range(n):
        mcom.get(pt.PRESENT_POS_SPEED_LOAD, [mid])

    print(('blackout for {} lasted {}{:.1f}ms{} ({} timeouts, {} communication'
           ' errors)').format(' and '.join(control.name for control, _, _ in writes), color.red, 1000*(end-start), color.end, timeouts, comerrors))
    return 1000*(end-start)

def test_blackout(writes, dur):
    """Verifies that a timeout of `dur` is sufficient to avoid errors"""
    global mid
    for i in range(n):
        mcom.get(pt.PRESENT_POS_SPEED_LOAD, [mid])

    for control, _, values in writes:
        if control == pt.ID:
            mcom.change_id(mid, values[0])
            mid = values[0]
        else:
            mcom.set(control, [mid], (values,))
    time.sleep(dur/1000.0)

    # should pass without error
    for i in range(n):
        mcom.get(pt.PRESENT_POS_SPEED_LOAD, [mid])


id_def       = mcom.motormems[mid][pt.ID]
max_temp_def = mcom.motormems[mid][pt.HIGHEST_LIMIT_TEMPERATURE]
min_volt_def = mcom.motormems[mid][pt.LOWEST_LIMIT_VOLTAGE]
max_volt_def = mcom.motormems[mid][pt.HIGHEST_LIMIT_VOLTAGE]

testcases = [[(pt.ID, [2], [4])],
             [(pt.CCW_ANGLE_LIMIT, [15], [2])],
             [(pt.CCW_ANGLE_LIMIT, [1003], [1020])],
             [(pt.CCW_ANGLE_LIMIT, [15], [2]), (pt.CCW_ANGLE_LIMIT, [1003], [1020])],
             [(pt.ANGLE_LIMITS, [10, 1000], [0, 1023])],
             [(pt.RETURN_DELAY_TIME, [10], [0])],
             [(pt.HIGHEST_LIMIT_TEMPERATURE, [max_temp_def-1], [max_temp_def])],
             [(pt.LOWEST_LIMIT_VOLTAGE, [min_volt_def+1], [min_volt_def])],
             [(pt.HIGHEST_LIMIT_VOLTAGE, [max_volt_def-1], [max_volt_def])],
             [(pt.VOLTAGE_LIMITS, [min_volt_def+1, max_volt_def-1], [min_volt_def, max_volt_def])],
             [(pt.MAX_TORQUE, [512], [1])],
             [(pt.STATUS_RETURN_LEVEL, [2], [1])],
             [(pt.ALARM_LED, [127], [37])],
             [(pt.ALARM_SHUTDOWN, [127], [37])],
             [(pt.ANGLE_LIMITS, [10, 1000], [0, 1023]), (pt.ALARM_SHUTDOWN, [127], [37])],
             [(pt.ANGLE_LIMITS, [10, 1000], [0, 1023]), (pt.MAX_TORQUE, [500], [512]), (pt.STATUS_RETURN_LEVEL, [1], [1])],
            ]

for writes in testcases:
    dur = blackout_duration(writes)
    test_blackout(writes, dur)

mcom.sio.close()
mcom.close()
