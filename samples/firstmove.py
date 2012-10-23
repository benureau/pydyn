import debugenv # only to debug local (not installed) pypot version

import sys, time

import pypot
import pypot.robot

if len(sys.argv) == 2:
    motor_id = int(sys.argv[1])
else:
    print('usage: firstmove.py motor_id')
    exit(1)

robot = pypot.robot.Robot(motor_range = [motor_id, motor_id], timeout = 0.02)

motor = robot.m_by_id[motor_id]

print motor.speed
#print robot.motors[96].speed = 20.0
motor.compliant = False

# robot.goto(96, 299, max_speed = 150)
# print motor.goal_position
# time.sleep(2.5)
# robot.goto(96, 1, max_speed = 150)
# print motor.goal_position
# time.sleep(2.5)

motor.position = 150.0
time.sleep(1.0)
motor.position = 100.0
time.sleep(1.0)
motor.position = 200.0
time.sleep(1.0)
motor.position = 150.0
time.sleep(1.0)


#motion = robot.sinus2(6, 150, 40, period = 2.0, duration = 10)
#motion = robot.sinus(94, 150, 40, period = 2.0, duration = 10)
#motion.join()
