import unittest

import env
from pydyn.ios.serialio import packet
from pydyn.refs import protocol as pt

class TestPackets(unittest.TestCase):

    def test_header(self):
        with self.assertRaises(AssertionError):
            packet.check_header(0, [255])
        with self.assertRaises(AssertionError):
            packet.check_header(0, [255, 254, 0, 2])
        with self.assertRaises(AssertionError):
            packet.check_header(0, [255, 254, 1, 2])

    def test_ping_packet(self):
        p = packet.InstructionPacket(3, pt.PING)
        self.assertEqual(p.mid, 3)
        self.assertEqual(p.instruction, pt.PING)
        self.assertEqual(p.length, len(p.params) + 2)
        self.assertTrue(len(p) == p.length + 4 == 6)

    def test_write_packet(self):
        p = packet.InstructionPacket(4, pt.WRITE_DATA, [pt.ANGLE_LIMITS.addr, 34, 0, 68, 0])
        self.assertEqual(list(p.data), [255, 255, 4, 7, 3, 6, 34, 0, 68, 0, 133])

if __name__ == '__main__':
    unittest.main()
