import sys, time
import env
import pydyn

ms = pydyn.MotorSet(timeout=4)
for p, m in zip(sys.argv[1:], ms.motors):
    if p != '_':
        m.position = float(p)

time.sleep(1.0)
