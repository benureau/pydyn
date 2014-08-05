from __future__ import print_function, division
import unittest
import pickle

import env
from pydyn.refs import protocol as pt
from pydyn.ios.serialio import packet
from pydyn.ios.serialio import serialcom


class TestComExceptions(unittest.TestCase):

    def test_comexc(self):
        #temp = tempfile.TemporaryFile()
        p = packet.InstructionPacket(4, pt.WRITE_DATA, [pt.ANGLE_LIMITS.addr, 34, 0, 68, 0])

        exc = serialcom.CommunicationError('bla', p, p)
        s = pickle.dumps(exc)
        exc2 = pickle.loads(s)

        self.assertEqual(exc, exc2)
        self.assertEqual(exc.inst_packet, exc2.inst_packet)


if __name__ == '__main__':
    unittest.main()
