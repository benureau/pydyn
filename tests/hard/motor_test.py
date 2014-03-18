import time

import env
import pydyn

from pydyn.refs import protocol as pt

ms = pydyn.MotorSet(timeout=10, latency=1, verbose=True)
time.sleep(0.01)

sleep = 0.1
ms.compliant = True
for motor in ms.motors:
    motor.angle_limits = (-150, 150)
    motor.max_torque = 50
    motor.status_return_level = 1
    motor.torque_limit = 50

ms.compliant = False
for motor in ms.motors:
    motor.position = -10
    time.sleep(sleep)
    motor.position =  10
    time.sleep(sleep)
    motor.position = 0
    time.sleep(sleep)

ms.compliant = True
time.sleep(0.01)
