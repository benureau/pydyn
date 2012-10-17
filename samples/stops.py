import debugenv # only to debug local (not installed) pypot version
"""An example to illustrate suspend, resume, stop and join methods."""

import time

import pypot
import pypot.robot

robot = pypot.robot.SimpleRobot(motor_range = [91, 96], timeout = 0.02)

robot.compliant = False
robot.speed = 30

robot.linear(150.0)
print "Going to position 150.0"
time.sleep(1.0)

print "Suspending motion"
robot.suspend()
time.sleep(1.0)

exit(0)

print "Resuming motion"
robot.resume()
time.sleep(1.0)

print "Stopping motion"
robot.stop()
time.sleep(1.0)

robot.linear(150.0)
print "Starting a new motion to 150.0"

print "Waiting for motion to finish..."
robot.join()
print "Finished !"



