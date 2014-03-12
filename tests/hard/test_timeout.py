"""Test the serial I/O"""

import unittest
import time

import env
from pydyn.ios.serialio import serialio

class TestSerialIO(unittest.TestCase):

    def test_timeout(self):
        n, timeout = 100, 5

        sio = serialio.Serial(device_type='USB2Serial', latency=1, timeout=timeout)

        a = time.time()
        for i in range(n):
            x = sio.read(10)
            assert x == bytearray(b'')
        d = time.time() - a

        self.assertTrue(n*timeout <= 1000*d <= n*(timeout+3))


if __name__ == '__main__':
    unittest.main()
