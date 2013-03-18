import debugenv # only to debug local (not installed) pydyn version

import sys, time

import pydyn
import pydyn.robot

robot = pydyn.robot.Robot(motor_range = [91, 96], timeout = 0.02)

robot.compliant = False
robot.speed = 30
robot.position = 150.0

print robot.position

robot.close()

