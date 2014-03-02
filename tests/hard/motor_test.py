import time

import env
import pydyn

ms = pydyn.MotorSet(verbose=True)
time.sleep(0.01)

sleep = 0.1
ms.compliant = True
for motor in ms.motors:
    motor.angle_limits_bytes = (0, 1023)
    motor.max_torque = 50
    motor.status_return_level = 1

ms.compliant = False
for motor in ms.motors:
#    print(motor.eeprom_desc())
#    print(motor.ram_desc())
    motor.position = 0
    time.sleep(sleep)
    motor.position = 10
    time.sleep(sleep)
    motor.position = 0
    time.sleep(sleep)

ms.compliant = True
time.sleep(0.01)
ms.close()
time.sleep(0.01)
