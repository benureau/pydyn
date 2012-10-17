import debugenv # only to debug local (not installed) pypot version

import time
import pypot
import pypot.robot

robot = pypot.robot.Robot(motor_range = [91, 96], timeout = 0.02)

motor = robot.m_by_id[94]

print motor.speed
#print robot.motors[96].speed = 20.0
motor.compliant = False

# robot.goto(96, 299, max_speed = 150)
# print motor.goal_position
# time.sleep(2.5)
# robot.goto(96, 1, max_speed = 150)
# print motor.goal_position
# time.sleep(2.5)

motion = robot.sinus2(94, 150, 40, period = 2.0, duration = 10)
#motion = robot.sinus(94, 150, 40, period = 2.0, duration = 10)
motion.join()
