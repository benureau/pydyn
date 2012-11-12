import debugenv # only to debug local (not installed) pypot version

import sys, time

import pypot
import pypot.robot

robot = pypot.robot.Robot(motor_range = [91, 96], timeout = 0.02)

robot.compliant = False
robot.speed = 30
robot.position = 150.0

print robot.position

robot.close()

