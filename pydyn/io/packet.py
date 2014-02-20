"""
Dynamixel Serial Packets

This package is not stringy, eg.
    InstructionPacket(2, 2, ())
No verification about is done about instructions. This makes it very flexible
and easily misused. All the checking is done in the motor com part.
"""

HEADER_SIZE = 4
def check_header(motor_id, data):
    """Check that an header is consistent"""
    data = bytearray(data)
    assert len(data) == 4 and data[0] == data[1] == 255 and motor_id == data[2]

class DynamixelPacketError(Exception):
    """Thrown when a status packet is not consistent"""
    def __init__(self, data, msg):
        """Details about the error in the msg"""
        Exception.__init__(self)
        self.data = data
        self.msg = msg


class RawPacket(object):
    'Base packet class : no header, no checksum, for debug and crazy stuff.'

    def __init__(self, data):
        """Instanciate a packet
        :param data:   string or list of integer
        """
        self.data = bytearray(data)

    def __len__(self):
        return len(self.data)


class Packet(RawPacket):

    @staticmethod
    def checksum(data):
        return 255-(sum(data)%256)

    @property
    def motor_id(self):
        return self.data[2]

    @property
    def length(self):
        return self.data[3]

    @property
    def params(self):
        return self.data[5:-1]


class InstructionPacket(Packet):
    """
    Layout is (details: http://support.robotis.com/en/product/dxl_main.htm):
    [0xff, 0xff, id, length, instruction, param1, param2, ... paramN, checksum]
    """
    def __init__(self, motor_id, instruction, params=()):
        data = [0xff, 0xff, motor_id, len(params)+2, instruction]
        data.extend(params)
        data.append(self.checksum(data[2:]))
        Packet.__init__(self, data)

    @property
    def instruction(self):
        return self.data[4]


class StatusPacket(Packet):
    """
    Layout is (details: http://support.robotis.com/en/product/dxl_main.htm):
    [0xff, 0xff, id, length, error, param1, param2, ... paramN, checksum]
    """

    def __init__(self, data):
        Packet.__init__(self, data)
        try: # checking consistency
            if self.length != len(self.data) - 4:
                raise DynamixelPacketError(self.data,
                      "length should be {}, but is {}.".format(self.length, len(self.data)-4))
            if self.data[-1] == self.checksum(data[2:-1]):
                raise DynamixelPacketError(self.data,
                      "checksum should be {}, but is {}.".format(self.checksum(data[2:-1]), self.data[-1]))
        except (IndexError, AssertionError):
            raise DynamixelPacketError(self.data, "corrupted packet")

    @property
    def error(self):
        return self.data[4]
