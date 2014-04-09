import pydyn

mset = pydyn.MotorSet(timeout=10)
for m in mset.motors:
    print m.eeprom_desc()
