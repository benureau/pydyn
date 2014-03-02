"""Test the code using the fake com module"""

import unittest
import time

import env
from pydyn.ios.fakeio import fakecom
from pydyn.dynamixel import controller

class TestFake(unittest.TestCase):

    def setUp(self):
        self.mcom = fakecom.FakeCom()
        self.mcom._add_motor(3,  'AX-12')
        self.mcom._add_motor(17, 'AX-12')

    def tearDown(self):
        self.mcom.close()

    def test_motor(self):
        """DynamixelController"""
        ctrl = controller.DynamixelController(self.mcom)
        mids = ctrl.discover_motors(verbose=False)
        ctrl.load_motors(mids)
        ctrl.start()
        m = ctrl.motors[0]

        time.sleep(0.1)

        # position checks
        m.goal_position = 0
        time.sleep(0.1)
        self.assertEqual(m.goal_position_bytes, 512)

if __name__ == '__main__':
    unittest.main()
