import unittest

import env
from pydyn.ios.serialio import packet
from pydyn.refs import protocol as pt

class TestPackets(unittest.TestCase):

    def test_connection(self):
        p = packet.InstructionPacket(3, pt.PING)
        self.assertEqual(p.mid, 3)
        self.assertEqual(p.instruction, pt.PING)
        self.assertEqual(p.length, len(p.params) + 2)
        self.assertTrue(len(p) == p.length + 4 == 6)

    def test_header(self):
        with self.assertRaises(AssertionError):
            packet.check_header(0, [255])
        with self.assertRaises(AssertionError):
            packet.check_header(0, [255, 254, 0, 2])
        with self.assertRaises(AssertionError):
            packet.check_header(0, [255, 254, 1, 2])


if __name__ == '__main__':
    unittest.main()
