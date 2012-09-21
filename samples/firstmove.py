import time
import os, sys
sys.path.insert(0, (os.path.join(os.getcwd(), '..')))

import pypot
import pypot.robot
import pypot.robot.robot


robot = pypot.robot.robot.SimpleRobot(motor_range = [91, 96])

motor = robot.motors[96]

print motor.speed
#print robot.motors[96].speed = 20.0
motor.compliant = False

# robot.goto(96, 299, max_speed = 150)
# print motor.goal_position
# time.sleep(2.5)
# robot.goto(96, 1, max_speed = 150)
# print motor.goal_position
# time.sleep(2.5)

motion = robot.sinus(96, 150, 40, period = 2.0, duration = 10, max_speed = 200)
motion.join()
