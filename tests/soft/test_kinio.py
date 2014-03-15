"""Test the code using the fake com module"""

import unittest
import time

import env
from pydyn.ios.kinio import kinio
from pydyn.ios.kinio import kinmotor
from pydyn.ios.serialio import serialcom
from pydyn.dynamixel import controller

class TestKinIO(unittest.TestCase):

    def setUp(self):
        self.kio = kinio.KinSerial()
        self.mcom = serialcom.SerialCom(self.kio)

    def tearDown(self):
        self.mcom.close()

    def test_kinmotor_set(self):
        m13 = kinmotor.KinMotor('AX-12', 13)
        self.assertEqual(m13.motor.id, 13)

    def test_kinmotor_discover(self):
        m13 = kinmotor.KinMotor('AX-12', 13)
        m17 = kinmotor.KinMotor('AX-12', 17)
        c = kinio.KinCable(m13.ports[1], m17.ports[0])

        self.kio.connect(m13.ports[0])
        ctrl = controller.DynamixelController(self.mcom)
        mids = ctrl.discover_motors(verbose=False)
        self.assertEqual(len(mids), 2)
        self.assertEqual(set(mids), set([13, 17]))

    def test_kinmotor_ctrl(self):
        m13 = kinmotor.KinMotor('AX-12', 13)
        m17 = kinmotor.KinMotor('AX-12', 17)
        c = kinio.KinCable(m13.ports[1], m17.ports[0])

        self.kio.connect(m13.ports[0])
        ctrl = controller.DynamixelController(self.mcom)
        mids = ctrl.discover_motors(verbose=False)
        ctrl.load_motors(mids)
        ctrl.start()
        time.sleep(0.3)
        ctrl.close()

    def test_kinmotor_ctrl(self):
        _m13 = kinmotor.KinMotor('AX-12', 13)
        _m17 = kinmotor.KinMotor('AX-12', 17)
        c = kinio.KinCable(_m13.ports[1], _m17.ports[0])

        self.kio.connect(_m13.ports[0])
        ctrl = controller.DynamixelController(self.mcom)
        mids = ctrl.discover_motors(verbose=False)
        ctrl.load_motors(mids)
        ctrl.start()

        for motor in ctrl.motors:
            motor.angle_limits = (-150, 150)
            motor.max_torque = 50
            motor.status_return_level = 1
            motor.torque_limit = 50

        sleep = 0.2
        for motor in ctrl.motors:
            motor.position = -10

        time.sleep(sleep)
        for motor in ctrl.motors:
            self.assertTrue(-10.2 <= motor.position <= -9.8)
            motor.position =  10

        time.sleep(sleep)
        for motor in ctrl.motors:
            self.assertTrue(9.8 <= motor.position <= 10.2)

        ctrl.close()


if __name__ == '__main__':
    unittest.main()
