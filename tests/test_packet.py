import unittest

import env
from pydyn.io import packet
from pydyn.io import protocol as pt

class TestPackets(unittest.TestCase):

    def test_connection(self):
        p = packet.InstructionPacket(3, pt.PING)
        self.assertEqual(p.motor_id, 3)
        self.assertEqual(p.instruction, pt.PING)
        self.assertEqual(p.length, len(p.params) + 2)
        self.assertTrue(len(p) == p.length + 4 == 6)


if __name__ == '__main__':
    unittest.main()
