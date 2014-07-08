import pydyn

mset = pydyn.MotorSet(timeout=5)
for m in mset.motors:
    print m.ram_desc()
