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


if __name__ == '__main__':
    unittest.main()
