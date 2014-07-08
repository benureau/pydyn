import pydyn, time
mset = pydyn.MotorSet(timeout=5, latency=5)
# mset.max_torque = 100
# mset.return_delay_time = 0
# mset.status_return_level = 1
# time.sleep(1)
mset.position = 0
time.sleep(1)
