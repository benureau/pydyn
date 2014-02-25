"""Test the code using the fake com module"""

import unittest

import env
from pydyn.ios.fakeio import fakecom
from pydyn.dynamixel import controller

class TestFake(unittest.TestCase):

    def test_add_motors(self):
        mcom = fakecom.FakeCom()
        mcom._add_motor(3,  'AX-12')
        mcom._add_motor(17, 'AX-12')
        self.assertEqual([mid for mid in range(0, 253) if mcom.ping(mid)], [3, 17])

    def test_ctrl(self):
        """DynamixelController"""
        mcom = fakecom.FakeCom()
        mcom._add_motor(3,  'AX-12')
        mcom._add_motor(17, 'AX-12')
        ctrl = controller.DynamixelController(mcom)
        mids = ctrl.discover_motors(verbose=False)
        ctrl.load_motors(mids)
