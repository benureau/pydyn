from ..serialio import serialio
from .kinmotor import KinPort, KinCable

class KinSerial(serialio.Serial):

    def __init__(self):
        self.port = KinPort(self)
        self._input_buffer = bytearray()
        self.cable = None


    def connect(self, port):
        self.cable = KinCable(self.port, port)

    def receive(self, msg):
        self._input_buffer += bytearray(msg)

    @property
    def support_sync_read(self):
        return False # for the moment

    @property
    def timeout(self):
        """Return the read timeout (not the write one)"""
        return 0

    @timeout.setter
    def timeout(self, val):
        """Set the read timeout (not the write one)"""
        pass

    @property
    def baudrate(self):
        return 1000000

    @baudrate.setter
    def baudrate(self, val):
        pass

    @property
    def latency(self):
        return 0

    @latency.setter
    def latency(self, val):
        pass

    def _toss_mode(self):
        """Activate the toss mode in the CM-5 and CM-510."""
        pass

    def flush(self):
        pass

    def purge(self):
        """purge and discard read and write buffers"""
        self._input_buffer = bytearray()

    def write(self, data):
        """Write data on the serial port"""
        self.cable.transmit(self.port, data)

    def read(self, size):
        if size > len(self._input_buffer):
            raise Warning('insufficient data in input buffer')
        data = self._input_buffer[:size]
        self._input_buffer = self._input_buffer[size:]
        return data

    def close(self):
        self.cable = None