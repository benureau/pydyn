"""
A wrapper around pyserial and pyftdi.
Allow to use pyftdi if possible, and else pyserial.
"""
from __future__ import print_function, division
import platform
import os
import time
import re

# pyserial imports
import serial
try:
    import serial.tools.list_ports_vid_pid_osx_posix as list_ports
except ImportError:
    print("error: the pyserial installed version is not compatible.\n"
          "       Install the makerbot version of pyserial available at "
          "https://github.com/makerbot/pyserial.\n")
    exit()

# ftdi imports
try:
    from pyftdi.pyftdi import ftdi
    _pyftdi_available = True
except ImportError:
    _pyftdi_available = False

USB_DEVICES    = {'USB2Serial', 'USB2Dynamixel', 'USB2AX', 'CM-513', 'CM-700',
                  'CM-900', 'OpenCM9.04', 'CM-100', 'CM-100A'}
SERIAL_DEVICES = {'SerialPort', 'CM-5', 'CM-510'}

def available_ports():
    """
    Returns the available ports found.

    Use makerbot's pyserial for port discovery.
    """
    return list_ports.list_ports_by_vid_pid()

def filter_ports(ports, device_type, port_path=None, serial_id=None):
    """
    Filter ports description based on device_type, port, or serial_id:
    * POSIX compliant (Linux, *BSD, HURD):
        if the device is a CM-5 or CM-510 controller:
            /dev/ttyS*
        else, the device is assumed to be a usb/serial adaptater.
            /dev/ttyACM* and /dev/ttyUSB*
    * OS X:
        no serial port, only usb/serial adaptaters.
        /dev/cu.usbserial and /dev/tty.usbserial-*
    * Windows:
        not tested.
    """
    plat = platform.system()

    # looking for a specific port
    if port_path is not None:
        port_path = os.path.abspath(port_path)
        ports = [port for port in ports if port['port'] == port_path]
        assert len(ports) <= 1

    if serial_id is not None:
        ports = [port for port in ports if 'iSerial' in port and port['iSerial'] == serial_id]
        assert len(ports) <= 1

    if port_path is None and serial_id is None:

        if plat == 'Darwin':
            regex = re.compile('tty.Bluetooth.*')
            ports = [port for port in ports if regex.search(port[0]) is None]

        elif plat == 'Windows':
            # TODO
            pass

        else: # POSIX
            regex = re.compile('/dev/ttyS.*')
            if device_type in SERIAL_DEVICES:
                ports = [port for port in ports if regex.search(port[0]) is not None]
            elif device_type in USB_DEVICES:
                ports = [port for port in ports if regex.search(port[0]) is None]

    return ports




class Serial(object):
    """
    Serial API

    Offer a pyserial-like API that switch to pyftdi when the hardware supports it.
    """

    def __init__(self, port=None, device_type='Any', serial_id=None,
                 baudrate=1000000, timeout=20, latency=2,
                 **kwargs):
        """
        Create a serial port.

        If possible, create it using pyftdi, and set latency
        :param port:         if not None, should point to a valid /dev path
        :param device_type:  can be any of 'USB2Serial', 'USB2Dynamixel',
                             'USB2AX', 'CM-513', 'CM-700', 'CM-900',
                             'OpenCM9.04', 'CM-100', 'CM-100A', 'SerialPort',
                             'CM-5', 'CM-510'. Specific initialization are then
                             used for each hardware.
        :param serial_id:    in the case of multiple usbserial hardware, use
                             this parameter to specify with one to connect to.
                             (eg. Serial(serial_id='A4008aCD'))
        :param baudrate:     serial baudrate in bps
        :param timeout:      serial read timeout
        :param latency:      set the usb latency timer in ms. (only supported
                             in FTDI hardware)

        Remaining kwargs will be passed to pyserial initialization.
        """

        ports = available_ports()
        ports = filter_ports(ports, device_type, port, serial_id)
        port_desc = ports[0] # taking the first one

        self._device_type = device_type

        self._serial = None
        self._ftdi_ctrl = False
        if _pyftdi_available:
            try:
                vid = port_desc['VID']
                pid = port_desc['PID']
                devs = ftdi.Ftdi.find_all([(vid, pid)])
                pid, vid, serial_id, interface, description = devs[0]

                self._serial = ftdi.Ftdi()
                self._serial.open(pid, vid, interface,
                                  serial=serial_id, description=description)

                self._serial.set_baudrate(baudrate)
                self._serial.timeouts = (timeout, 0)
                self._serial.set_latency_timer(latency)
                self._ftdi_ctrl = True

            except KeyError:
                pass

        if self._serial is None: # pyftdi didn't succeed, trying pyserial
            port = port_desc['port']
            self._serial = serial.Serial(port, baudrate=baudrate,
                                         timeout=timeout/1000.0, **kwargs)

        if device_type in ['CM-5', 'CM-510']:
            self._toss_mode()

    def _toss_mode(self):
        """Activate the toss mode in the CM-5 and CM-510."""
        self.write('t\r')
        time.sleep(0.1)
        self.read(100)
        time.sleep(0.1)


    def __del__(self):
        """Destructor, close port when serial port instance is freed."""
        self.close()

    def flush(self):
        """Flush of file like objects. In this case, wait until all data is written."""
        self._serial.flush()

    def write(self, data):
        """Write data on the serial port"""
        if self._ftdi_ctrl:
            self._serial.write_data(data)
        else:
            self._serial.write(data)

    def read(self, size):
        """
        Read size bytes from the serial port.
        If a timeout is set it may return less characters as requested. With no timeout
        it will block until the requested number of bytes is read.
        """
        if self._ftdi_ctrl:
            self._serial.read_data(size)
        else:
            self._serial.read(size=size)

    def close(self):
        """Close the serial port"""
        self._serial.close()

