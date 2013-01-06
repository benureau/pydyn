import debugenv # only to debug local (not installed) pypot version

import time

import pypot
import pypot.robot

robot = pypot.robot.Robot(motor_range = [0, 20])
print robot.motors[0].torque_limit

robot.compliant = True
time.sleep(0.2)
robot.pose = 150
time.sleep(0.2)
robot.motors[0].torque_limit = 50.0
time.sleep(0.2)
robot.pose = 150
time.sleep(0.2)
print robot.motors[0].torque_limit
