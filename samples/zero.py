import env, pydyn, time
mset = pydyn.MotorSet()
mset.motors[0].compliant = True
time.sleep(0.2)
mset.motors[0].position = 30
time.sleep(0.2)
mset.motors[0].position = 0
time.sleep(0.2)