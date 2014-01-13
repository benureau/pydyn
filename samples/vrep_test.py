import time
import pydyn
pydyn.enable_vrep()

r = pydyn.robot.Robot(motor_range = [1, 6])
r._ctrl.io.sim.simStartSimulation()
time.sleep(1)
r.motors[5].compliant = False
r.motors[5].moving_speed = 10
print r.motors[5].moving_speed_raw

time.sleep(1)
print r.motors[5].moving_speed_raw
r.motors[5].position = 100

time.sleep(5)