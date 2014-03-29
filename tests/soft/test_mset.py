"""Test the code of MotorSet"""

import unittest
import time

import env
from pydyn.refs import protocol as pt
from pydyn.ios.fakeio import fakecom
from pydyn.ios.kinio import kinio
from pydyn.ios.kinio import kinmotor
from pydyn.ios.serialio import serialcom
from pydyn.dynamixel import controller
from pydyn.msets.msets import MotorSet

class TestFake(unittest.TestCase):

    def setUp(self):
        self.mcom = fakecom.FakeCom()
        self.mcom._add_motor(3,  'AX-12')
        self.mcom._add_motor(17, 'AX-12')
        self.ctrl = controller.DynamixelController(self.mcom)
        mids = self.ctrl.discover_motors(verbose=False)
        self.ctrl.load_motors(mids)
        self.ctrl.start()

    def tearDown(self):
        self.ctrl.close()

    def test_dir(self):
        ms = MotorSet(motors=self.ctrl.motors)
        self.assertTrue('compliant' in dir(ms))
        self.assertTrue('present_temperature_bytes' in dir(ms))
        self.assertTrue('request_write' in dir(ms))

    def test_iadd(self):
        ms = MotorSet(motors=self.ctrl.motors)
        ms.position += 10

    def test_empty(self):
        ms = MotorSet(motors=[])

        ms.pose = []
        with self.assertRaises(AttributeError):
            ms.punch

        with self.assertRaises(AttributeError):
            ms.punch = 50
        ms.punch = []
        ms.does_not_exist = []

    def test_get(self):
        ms = MotorSet(motors=self.ctrl.motors)
        self.assertEqual(len(ms.position), len(self.ctrl.motors))
        self.assertEqual(len(ms.position), 2)

        with self.assertRaises(AttributeError):
            ms.does_not_exist

    def test_set(self):
        ms = MotorSet(motors=self.ctrl.motors)
        ms.goal_position_bytes = 150
        time.sleep(0.05)
        self.assertEqual(ms.motors[0].goal_position_bytes, 150)
        self.assertEqual(ms.motors[1].goal_position_bytes, 150)

        ms.position_bytes = (100, 200)
        time.sleep(0.05)
        self.assertEqual(ms.motors[0].goal_position_bytes, 100)
        self.assertEqual(ms.motors[1].goal_position_bytes, 200)

        with self.assertRaises(AttributeError):
            ms.does_not_exist = 100

    def test_properties(self):
        ms = MotorSet(motors=self.ctrl.motors)
        ms.zero_pose = -150
        ms.zero_pose
        ms.pose = 0
        ms.pose

    def test_motor_properties(self):
        ms = MotorSet(motors=self.ctrl.motors)
        ms.moving_speed = 0
        ms.moving_speed


class TestKinMset(unittest.TestCase):

    def setUp(self):
        self.kio = kinio.KinSerial()
        self.mcom = serialcom.SerialCom(self.kio)
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

    def test_bundles(self):
        ms = MotorSet(motors=self.ctrl.motors)
        ms.cw_angle_limit = 150
        ms.angle_limits = (-150, 150)

    # def test_functions(self):
    #     ms = MotorSet(motors=self.ctrl.motors)
    #     ms.request_read(pt.PRESENT_POSITION)
    #     ms.request_read("present_speed")
    #     ms.request_write(pt.TORQUE_LIMIT, 512)
    #     ms.request_write("torque_limit_bytes", 512)
    #     time.sleep(0.05)
    #     #self.assertEqual(ms.pose, (0.0, 0.0))

if __name__ == '__main__':
    unittest.main()
