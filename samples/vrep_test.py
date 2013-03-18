import time
import pydyn
pydyn.enable_vrep()

r = pydyn.robot.Robot(motor_range = [1, 6])
r._ctrl.io.sim.simStartSimulation()
time.sleep(1)
r.motors[5].compliant = False
time.sleep(1)
r.motors[5].position = 50

time.sleep(5)