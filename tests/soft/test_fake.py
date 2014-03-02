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

    def test_add_motors(self):
        self.assertEqual([mid for mid in range(0, 253) if self.mcom.ping(mid)], [3, 17])

    def test_ctrl(self):
        """DynamixelController"""
        ctrl = controller.DynamixelController(self.mcom)
        mids = ctrl.discover_motors(verbose=False)
        ctrl.load_motors(mids)

    def test_motor(self):
        """DynamixelController"""
        ctrl = controller.DynamixelController(self.mcom)
        mids = ctrl.discover_motors(verbose=False)
        ctrl.load_motors(mids)
        m = ctrl.motors[0]

        # position checks
        m.goal_position_bytes = 0
        m.goal_position_bytes
        m.present_position_bytes
        with self.assertRaises(AttributeError):
            m.present_position_bytes = 0

        with self.assertRaises(ValueError):
            m.goal_position_bytes = 1024
        with self.assertRaises(ValueError):
            m.goal_position_bytes = -1
        with self.assertRaises(ValueError):
            m.max_torque_bytes = 1.3

    def test_setattr(self):
        ctrl = controller.DynamixelController(self.mcom)
        mids = ctrl.discover_motors(verbose=False)
        ctrl.load_motors(mids)
        m = ctrl.motors[0]

        with self.assertRaises(AttributeError):
            m.does_not_exists = 312

    def test_sync_motor(self):
        ctrl = controller.DynamixelController(self.mcom)
        mids = ctrl.discover_motors(verbose=False)
        ctrl.load_motors(mids)
        ctrl.start()

        m = ctrl.motors[0]
        m.goal_position_bytes = 312
        time.sleep(0.1)
        self.assertEqual(m.goal_position_bytes, 312)
        self.assertEqual(m.present_position, m.position)

        ctrl.stop()


if __name__ == '__main__':
    unittest.main()
