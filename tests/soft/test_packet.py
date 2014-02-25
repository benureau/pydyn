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


if __name__ == '__main__':
    unittest.main()
