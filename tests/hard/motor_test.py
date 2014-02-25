import time

import env
import pydyn

ms = pydyn.MotorSet(verbose=True)
time.sleep(0.01)

sleep = 0.1
ms.compliant = False
for motor in ms.motors:
    motor.position = 150
    time.sleep(sleep)
    motor.position = 160
    time.sleep(sleep)
    motor.position = 150
    time.sleep(sleep)

ms.compliant = True
time.sleep(0.01)
ms.close()
time.sleep(0.01)
