"""Test the code of MotorSet"""
from __future__ import print_function, division

import unittest
import time

import env
from pydyn.refs import protocol as pt
from pydyn.refs import exc
from pydyn.ios.fakeio import fakecom
from pydyn.ios.kinio import kinio
from pydyn.ios.kinio import kinmotor
from pydyn.ios.serialio import serialcom
from pydyn.dynamixel import controller
from pydyn.msets.msets import MotorSet


def _set(self, control, mids, valuess):
    raise self.MotorError(mids[0], ['FakeError'])

class TestKinMset(unittest.TestCase):

    def setUp(self):
        self.kio = kinio.KinSerial()
        self.mcom = serialcom.SerialCom(self.kio)
        self.mcom.set = _set.__get__(self.mcom, self.mcom.__class__)

        m13 = kinmotor.KinMotor('AX-12', 13)
        m17 = kinmotor.KinMotor('AX-12', 17)
        c = kinio.KinCable(m13.ports[1], m17.ports[0])
        self.kio.connect(m13.ports[0])
        self.ctrl = controller.DynamixelController(self.mcom)
        mids = self.ctrl.discover_motors(verbose=False)
        self.ctrl.load_motors(mids)
        self.ctrl.start()

    def tearDown(self):
        self.ctrl.close()

    def test_error(self):
        ms = MotorSet(motors=self.ctrl.motors)
        ms.cw_angle_limit = 150
        time.sleep(0.1)
        with self.assertRaises(exc.MotorError):
            ms.cw_angle_limit = 100

if __name__ == '__main__':
    unittest.main()
