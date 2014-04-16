from __future__ import print_function, division
import time
import sys

from pydyn.refs import protocol as pt
from pydyn.ios.serialio import serialio, serialcom, packet
from pydyn import color

sio = serialio.Serial(device_type='USB2Serial', latency=1, timeout=2)
print('I/O open on {}{}{}'.format(color.cyan, sio, color.end))
mcom = serialcom.SerialCom(sio)
detected = [mid for mid in range(0, 253) if mcom.ping(mid)]
print('detected: {}'.format(detected))

assert len(detected) == 1
mid = detected[0]

print('You are about to RESET the motor with id {}.'.format(mid))
print('Are you sure (y/N): ', end='')
s = raw_input()
if s != 'y':
    print('Exiting without changes')
    sys.exit()


print('Reseting...')
reset_packet = packet.InstructionPacket(mid, pt.RESET)
mcom._send_packet(reset_packet)

time.sleep(1.0)
print('Done.')
