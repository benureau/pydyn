import time
import os, sys
sys.path.insert(0, (os.path.join(os.getcwd(), '..')))

import pypot
import pypot.robot
import pypot.robot.robot


robot = pypot.robot.robot.SimpleRobot(motor_range = [91, 96])

print robot.motor_pos(96)
print robot.motors[96].speed
robot.motors[96].speed = 20.0
#print robot.motors[96].speed = 20.0
robot.motors[96].compliant = False

while True:
    motion = robot.goto(96, 300, 0.5)
    motion.join()
    time.sleep(0.5)
    motion = robot.goto(96, 0, 1.0)
    motion.join()
    time.sleep(0.5)

