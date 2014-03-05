# -*- coding: utf-8 -*-
"""
Motor communication module

Handle communication request of type :
    * get value of 'PRESENT_POSITION' of motor with id 13
    * set value of 'GOAL_POSITION' of motor 13 at 743

Notably, this module does do any checks on the values of the write requests,
excepts that it fits into the size of the motor adress.
It does not either do any conversion, this is done at high level.
"""

import threading
import itertools
#import atexit

from ...refs import protocol as pt
from ...refs import alarms as conv # the only conversion needed in I/O
from ...dynamixel import memory
from . import packet



# MARK: - Dxl Error

class CommunicationError(Exception):
    """Thrown when a status packet arrived corrupted."""
    def __init__(self, msg, inst_packet, status_packet):
        self.msg = msg
        self.inst_packet    = inst_packet
        self.status_packet = status_packet

    def __str__(self):
        return "CommunicationError('{}', {}, {})".format(self.msg, list(self.inst_packet.data), self.status_packet)

class TimeoutError(Exception):
    """Thrown when no status packet has arrived when timeout kicks in.

    The differences between Communication error and Timeout error allow to
    handle different case: while it is sometimes expected to get a timeout error
    (eg. ping non-existent motor), it is never good to get corrupted data.
    """
    def __init__(self, inst_packet):
        self.inst_packet    = inst_packet

    def __str__(self):
        return "TimemoutError({})".format(list(self.inst_packet.data))


class MotorError(Exception):
    def __init__(self, mid, alarms):
        self.mid = mid
        self.alarms = alarms

    def __str__(self):
        return 'Motor {} triggered alarm{}: {}'.format(self.mid,
                    's' if len(self.alarms) > 1 else '',
                    self.alarms if len(self.alarms) > 1 else self.alarms[0])


# MARK: - SerialCom class

class SerialCom(object):
    """
    This class handles the low-level communication with robotis motors.

    Using a USB communication device such as USB2DYNAMIXEL or USB2AX,
    you can open serial communication with robotis motors (MX, AX, RX)
    using communication protocols TTL for the first two and RS485 for the
    latter.

    This class handles low-level IO communication by providing access to the
    different registers in the motors.
    Users can use high-level access such as position, load, torque to control
    the motors.

    You can access two different area space of the dynamixel registers:
        * the EEPROM area
        * the RAM area

    When values are written to the EEPROM, they are conserved after cycling
    the power.
    The values written to the RAM are lost when cycling the power.

    In this modules, all values are bytes. Conversion to human values are made
    at higher level, through the motor interface.

    Also, this module does minimal checking about if values make sense or are legal.
    The Motor instance is responsible for that. It makes everything simpler here, and
    problems are catched earlier by the Motor instances. Plus, if you really want to
    go crazy experimenting on non-legal value, you can by accessing this level.

    .. warning:: When writing on EEPROM registers the motor enters a "busy" mode
                 will no respond correctly to requests for roughly 20ms times the
                 number of register written (not all register are equal, check the
                 eeprom_blackout.py sample for more details).
    """
    CommunicationError = CommunicationError
    TimeoutError = TimeoutError
    MotorError = MotorError

    # __open_ports = [] # TODO: unified interface

    def __init__(self, sio, verbose=True, **kwargs):
        """
        :param sio:  a functional, opened serial io instance.

        :raises: IOError (when port is already used)
        """
        # if port in self.__open_ports:
        #     raise IOError('Port already used (%s)!' % (port))

        self.sio = sio
        self.verbose = verbose
        # self.__open_ports.append(port)

        self._lock = threading.RLock()

        self.motormems = {}

    @property
    def support_sync_read(self):
        return self.sio.support_sync_read

    def close(self):
        try:
            #self.__open_ports.remove(self.sio.port)
            self.sio.close()
        except Exception:
            pass

    def purge(self):
        """Purge the IO port. Useful after an error."""
        self.sio.purge()

    def __del__(self):
        """ Automatically closes the serial communication on destruction. """
        self.close()

    # MARK: - Motor general functions

    def ping(self, mid):
        """Pings the motor with the specified id.

        :param int mid: specified motor id [0-253]
        :return: bool
        :raises: ValueError if the motor id is out of the possible ids range.
        """
        if not 0 <= mid <= 253:
            raise ValueError('Motor id must be in [0, 253]')

        ping_packet = packet.InstructionPacket(mid, pt.PING)

        try:
            self._send_packet(ping_packet)
            return True

        except TimeoutError:
            return False
        except CommunicationError as e:
            if (list(e.status_packet) == [0]):
                if self.verbose:
                    e.msg = 'status packet received after ping was [0]; it can indicate lack of power to the motors.'
                    print('warning: {}'.format(e.msg))
            return False

    def ping_broadcast(self):
        """Do a ping on the broadcast id.

        In some situations, everything goes well, and every motor respond in order, then
        every id are effectively scanned with only one packet.
        If no error is detected returns the motors ids, else return the empty list.

        .. warning: even when no error is detected, in some situation, this method will
                    reports motors that don't exist. This can be mitigated by doing
                    broadcast_ping() twice and using the second result.
        """

        timeout_bak = self.sio.timeout
        try:
            self.sio.timeout = 400

            ping_packet = packet.InstructionPacket(pt.BROADCAST, pt.PING)
            self._send_packet(ping_packet, receive=False)

            data = self.sio.read(6*253) # We get that ourselves
            assert len(data) % 6 == 0, "broadcast_ping data is of lenght {}".format(len(data))
            motors = []
            for i in range(int(len(data)/6)):
                packet.StatusPacket(data[6*i:6*(i+1)])
                motors.append(ord(data[6*i+2]))
        except (packet.PacketError, AssertionError):
            #import traceback
            #traceback.print_exc()
            raise IOError
        finally:
            self.sio.timeout = timeout_bak

        return motors

    def read(self, mid, addr, size):
        """
        Read arbitrary data from a motor

        :param addr:  where to read data in the memory.
        :param size:     how much from adress.
        :return:         list of integers with asked size
        """
        inst_packet = packet.InstructionPacket(mid, pt.READ_DATA, (addr, size))
        status_packet = self._send_packet(inst_packet)
        return status_packet.params

    def create(self, mids):
        """
        Load the motors memory.

        :param mids:  list of the motor ids to create.
        :return: instances of DynamixelMemory

        .. note:: if a memory already exist, it is recreated anyway.
        """
        mmems = []

        for mid in mids:
            mmem = memory.DynamixelMemory(mid)
            self.motormems[mmem.id] = mmem
            self.get(pt.EEPROM, [mid])
            self.get(pt.RAM,    [mid])

            mmems.append(mmem)

        return mmems

    # MARK Parameter based read/write

    def set(self, control, mids, valuess):
        """Send a write instruction and update memory

        :param control:  the control involved
        :param mids:     ids of motors. If more than one, do a sync_write.
        :param valuess:  list of sequence of values. list length should be
                         the same as mids, each sequence shape should
                         match the control.sizes parameter.
        """
        #print('set({})'.format(control.name))
        assert len(mids) > 0
        if len(mids) > 1 and sum(control.sizes) <= 6:
            self._send_sync_write_packet(control, mids, valuess)
        else:
            for mid, values in zip(mids, valuess):
                self._send_write_packet(control, mid, values)

    def get(self, control, mids):
        """Send a read instruction and update memory

        :param control:  the control involved
        :param mids:     ids of motors. If more than one, and the io supports
                         it, do a sync write.
        """
        #print('get({})'.format(control.name))
        assert len(mids) > 0
        if len(mids) == 1:
            self._send_read_packet(control, mids[0])
        else:
            if (self.support_sync_read
                and sum(control.sizes) <= 6 and len(mids) <= 30): # TODO split in several sync_read instead of reverting to read packets
                self._send_sync_read_packet(control, mids)
            else:
                for mid in mids:
                    self._send_read_packet(control, mid)

    # MARK - Special cases

    def change_id(self, mid, new_mid):
        """
        Changes the id of the motor.

        Each motor must have a unique id on the bus.
        The range of possible ids is [0, 253].

        :param int mid:      current motor id
        :param int new_mid:  new motor id
        :raises:  ValueError when the id is already taken
        """
        if (mid != new_mid and
            (new_mid in self.motormems or self.ping(new_mid))):
            raise ValueError('id %d already used' % (new_mid))

        self._send_write_packet(pt.ID, mid, [new_mid])

        self.motormems[new_mid] = self.motormems.pop(mid)

    def get_status_return_level(self, mid):
        """
        Returns the level of status return.

        Threre are three levels of status return:
            * 0 : no return packet
            * 1 : return status packet only for the read instruction
            * 2 : always return a status packet

        :param int mid:  specified motor id [0-253]

        .. note::  if the EEPROM has been properly loaded, executing this
                   is a waste of a good serial packet.
        """

        try:
            status_return_level = self._send_read_packet(pt.STATUS_RETURN_LEVEL, mid)
            self.motormems[mid][pt.STATUS_RETURN_LEVEL] = status_return_level
        except TimeoutError as e:
            if self.ping(mid):
                self.motormems[mid][pt.STATUS_RETURN_LEVEL] = 0
            else:
                raise e

    # MARK: - Low level communication

    def _send_packet(self, inst_packet, receive=True):
        """Send a packet and handle the (eventual) reception"""
        with self._lock:
            n = self.sio.write(bytes(inst_packet.data))
            if n != len(inst_packet):
                raise CommunicationError('Packet not correctly sent', inst_packet, None)

            if receive:
                data = self.sio.read(packet.HEADER_SIZE)
                if len(data) < packet.HEADER_SIZE:
                    data += self.sio.read(packet.HEADER_SIZE-len(data))
                if len(data) == 0:
                    raise TimeoutError(inst_packet)

                try:
                    packet.check_header(inst_packet.mid, data)
                except AssertionError as e:
                    self.sio.purge()
                    raise CommunicationError(e.args[0],
                                             inst_packet, list(bytearray(data)))

                try:
                    data += self.sio.read(data[3])
                    status_packet = packet.StatusPacket(data)

                except packet.PacketError as e:
                    self.sio.purge()
                    raise CommunicationError(e.msg, inst_packet,
                                             list(bytearray(data)))

                if status_packet.error != 0:
                    alarms = conv.bytes2_alarm_names(status_packet.error)
                    if len(alarms):
                        raise MotorError(status_packet.mid, alarms)

                return status_packet

    def _update_memory(self, control, mid, values):
        """Update the memory of the motors"""
        offset = 0
        for size, value in zip(control.sizes, values):
            self.motormems[mid][control.addr+offset] = value
            offset += size
        self.motormems[mid].update() # could be more selective, but this would be useless optimization.

    def _send_read_packet(self, control, mid):
        """Send a read packet and update memory if successful."""
        if self.motormems[mid].status_return_level == 0:
            print(('warning, status_return_level of motor {} is at 0, '
                   'no reads possible').format(mid))

        else:
            read_packet = packet.InstructionPacket(mid, pt.READ_DATA, (control.addr, sum(control.sizes)))
            status_packet = self._send_packet(read_packet, receive=True)

            if status_packet:
                values = self._to_values(control, status_packet.params)
                self._update_memory(control, mid, values)
                return values

    def _send_sync_read_packet(self, control, mids):
        raise NotImplementedError

    def _send_write_packet(self, control, mid, values):
        """Send a write packet and update memory optimistically if no error."""
        params = self._to_params(control, values)

        write_packet = packet.InstructionPacket(mid, pt.WRITE_DATA, [control.addr] + params)

        self._send_packet(write_packet, receive=self.motormems[mid].status_return_level == 2)
        self._update_memory(control, mid, values)

    def _send_sync_write_packet(self, control, mids, valuess):
        """
        Parameters layout is (details: http://support.robotis.com/en/product/dynamixel/communication/dxl_instruction.htm):
        [start addr, length of data to write, id0, param0id0, param1id1, ...,
                                                 id1, param0id1, param2id2, ...]
        """
        params = itertools.chain.from_iterable((
                    [mid]+self._to_params(control, values)
                    for mid, values in zip(mids, valuess)))

        sync_write_packet = packet.InstructionPacket(pt.BROADCAST, pt.SYNC_WRITE, [control.addr]+list(params))
        self._send_packet(sync_write_packet, receive=False)
        for mid, values in zip(mids, valuess):
            self._update_memory(control, mid, values)

    # MARK : - Parameter encoding/decoding

    @staticmethod
    def _to_values(control, params):
        """
        Transform parameters of a status packet in one and two bytes values
        """
        assert sum(control.sizes) == len(params), "{} should have length {} but has {}".format(list(values), sum(control.sizes), len(values))
        itp, values = params.__iter__(), []
        for s in control.sizes:
            values.append(next(itp))
            if s == 2:
                values[-1] += next(itp) << 8
        return values

    @staticmethod
    def _to_params(control, values):
        """
        Transform one bytes and two bytes values into parameters for
        an instruction packet
        """
        assert len(control.sizes) == len(values), "{} and {} don't have the same length".format(control.sizes, list(values))
        params = []
        for v, s in zip(values, control.sizes):
            if s == 1:
                params.append(v)
            elif s == 2:
                params.append(v % 256)
                params.append(v >> 8)
        return params

