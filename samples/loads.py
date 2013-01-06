import debugenv # only to debug local (not installed) pypot version

import time

import pypot
import pypot.robot

robot = pypot.robot.Robot(motor_range = [0, 20], timeout = 0.03)

robot.m_by_id[17].compliant = False

motion = robot.constantspeed(-700, motor_ids = 17, duration = 20)[0]

while motion._alive:
    print robot.m_by_id[17].current_load
    time.sleep(0.2)

motion = robot.constantspeed(0, motor_ids = 17, duration = 0.3)[0]
motion.join()