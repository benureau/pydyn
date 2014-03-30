from __future__ import print_function, division
import unittest
import platform

import env
from pydyn.ios.serialio import serialio

linux_ports = [{'blob': ('/dev/ttyS0', 'ttyS0', 'n/a'), 'port': '/dev/ttyS0'}, 
               {'VID': 24577, 'PID': 1027, 'iSerial': u'AD01UYPC', 'interface': 1, 'port': u'usbftdi:AD01UYPC', 'desc': u'FT232R USB UART'}]

system_bak = platform.system

class TestSerialIOLinux(unittest.TestCase):

    def setUp(self):
        def system():
            return 'Linux'
        platform.system = system

    def tearDown(self):
        platform.system = system_bak

    def test_filter_port_linux(self):
        print(serialio.filter_ports(linux_ports, device_type='USB2Serial'))



if __name__ == '__main__':
    unittest.main()
