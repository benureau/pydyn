"""
Dynamixel Serial Packets

This package is not stringy, eg.
    InstructionPacket(2, 2, ())
No verification about is done about instructions. This makes it very flexible
and easily misused. All the checking is done in the motor com part.
"""

HEADER_SIZE = 4
def check_header(mid, data):
    """Check that an header is consistent"""
    data = bytearray(data)
    assert len(data) == 4, "header size ({}) is wrong (!=4)".format(len(data))
    assert data[0] == data[1] == 255, "header prefix ({}) is wrong (!= [255, 255])".format(list(data[:2]))
    assert mid == data[2], "mid ({}) is wrong (!={})".format(data[2], mid)

class PacketError(Exception):
    """Thrown when a status packet is not consistent"""
    def __init__(self, msg, data):
        """Details about the error in the msg"""
        Exception.__init__(self)
        self.data = data
        self.msg = msg

    def __str__(self):
        return "PacketError('{}', {})".format(self.msg, list(self.data))

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
    def mid(self):
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
    def __init__(self, mid, instruction, params=()):
        data = [0xff, 0xff, mid, len(params)+2, instruction]
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
                raise PacketError("length should be {}, but is {}.".format(
                                  self.length, len(self.data)-4), self.data)
            if self.data[-1] != self.checksum(self.data[2:-1]):
                raise PacketError("checksum should be {}, but is {}.".format(
                                  self.checksum(self.data[2:-1]), self.data[-1]), self.data)
        except (IndexError, AssertionError):
            raise PacketError("corrupted packet", self.data)

    @property
    def error(self):
        return self.data[4]
