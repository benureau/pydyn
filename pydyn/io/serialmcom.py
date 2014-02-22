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

from . import protocol as pt
from . import packet
from ..dynamixel import memory

from ..dynamixel import alarms as conv # the only conversion needed in I/O


# MARK: - Dxl Error

class CommunicationError(Exception):
    """Thrown when a status packet arrived corrupted"""
    def __init__(self, msg, inst_packet, status_packet):
        self.msg = msg
        self.inst_packet    = inst_packet
        self.status_packet = status_packet

    def __str__(self):
        return "CommunictionError('{}', {}, {})".format(self.msg, list(self.inst_packet.data), self.status_packet)

class TimeoutError(Exception):
    """Thrown when no status packet has arrived when timeout kicks in.

    The differences between Communication error and Timeout error allow to
    handle different case: while it is sometimes expected to get a timeout error
    (eg. ping non-existent motor), it is never good to get corrupted data.
    """
    def __init__(self, inst_packet):
        self.inst_packet    = inst_packet

class MotorError(Exception):
    def __init__(self, motor_id, alarms):
        self.motor_id = motor_id
        self.alarms = alarms

    def __repr__(self):
        return 'Motor %d triggered alarm%s: %s' % (self.motor_id,
                                                   's' if len(self.alarms) > 1 else '',
                                                   self.alarms if len(self.alarms) > 1 else self.alarms[0])


# MARK: - DynamixelComSerial class

class DynamixelComSerial(object):
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

    When values are written to the EEPROM, they are conserved after cycling the power.
    The values written to the RAM are lost when cycling the power.

    In this modules, all values are raw, integer values. Conversion to human values
    are made at higher level, through the motor interface.

    Also, this module does minimal checking about if values make sense or are legal.
    The Motor instance is responsible for that. It makes everything simpler here, and
    problems are catched earlier by the Motor instances. Plus, if you really want to
    go crazy experimenting on non-legal value, you can by accessing this level.

    .. warning:: When accessing EEPROM registers the motor enters a "busy" mode and
                 should not be accessed before about 100ms. [#TODO: is that really true ?]

    """
    CommunicationError = CommunicationError

    # __open_ports = [] # TODO: unified interface

    def __init__(self, sio, **kwargs):
        """
        :param sio:  a functional, opened serial io instance.

        :raises: IOError (when port is already used)
        """
        # if port in self.__open_ports:
        #     raise IOError('Port already used (%s)!' % (port))

        self.sio = sio
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

    def __del__(self):
        """ Automatically closes the serial communication on destruction. """
        self.close()

    # MARK: - Motor general functions

    def ping(self, motor_id):
        """Pings the motor with the specified id.

        :param int motor_id: specified motor id [0-253]
        :return: bool
        :raises: ValueError if the motor id is out of the possible ids range.
        """
        if not 0 <= motor_id <= 253:
            raise ValueError('Motor id must be in [0, 253]!')

        ping_packet = packet.InstructionPacket(motor_id, pt.PING)

        try:
            self._send_packet(ping_packet)
            return True

        except TimeoutError:
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

    def scan(self, ids=range(254)):
        """
            Finds the ids of all the motors connected to the bus.

            :param list ids: the range of ids to search
            :return: list of ids found

            """
        return [mid for mid in ids if self.ping(mid)]


    def read(self, motor_id, addr, size):
        """
        Read arbitrary data from a motor

        :param addr:  where to read data in the memory.
        :param size:     how much from adress.
        :return:         list of integers with asked size
        """
        inst_packet = packet.InstructionPacket(motor_id, pt.READ_DATA, (addr, size))
        status_packet = self._send_packet(inst_packet)
        return status_packet.params

    def create(self, motor_ids):
        """
        Load the motors memory.

        :param list ids, the ids to create.
        :return: instances of DynamixelMemory

        .. warning:: we assume the motor id has been checked by a previous ping.
        .. note:: if a memory already exist, it is recreated anyway.
        """

        mmems = []

        for motor_id in motor_ids:
            # reading eeprom, ram
            raw_eeprom = self.read(motor_id, 0, 24)
            raw_ram    = self.read(motor_id, 24, 26)
            mmem = memory.DynamixelMemory(raw_eeprom, raw_ram)

            # reading extra ram (if necessary)
            extra_addr = mmem.extra_addr()
            if extra_addr is not None:
                addr, size = extra_addr
                raw_extra = self.read(motor_id, addr, size)
                mmem._process_extra(raw_extra)

            # registering the motor memory to the io
            self.motormems[mmem.id] = mmem
            mmems.append(mmem)

        return mmems

    def read_ram(self, motor_id):
        mmem    = self.motormems[motor_id]
        raw_ram = self.read(motor_id, 24, mmem.last_addr() - 24 + 1)
        mmem._process_raw_ram(raw_ram)

        extra_addr = mmem.extra_addr()
        if extra_addr is not None:
            addr, size = extra_addr
            mmem.process_extra(raw_ram[addr, addr+size])


    # MARK Parameter based read/write


    def set(self, control, motor_ids, valuess):
        """Send a write instruction and update memory

        :param control:    the control involved
        :param motor_ids:  ids of motors. If more than one, do a sync_write.
        :param valuess:    list of sequence of values. list length should be
                           the same as motor_ids, each sequence shape should
                           match the control.sizes parameter.
        """
        assert len(motor_ids) > 0
        if len(motor_ids) > 1 and sum(control.sizes) <= 6:
            self._send_sync_write_packet(control, motor_ids, valuess)
        else:
            for motor_id, values in zip(motor_ids, valuess):
                self._send_write_packet(control, motor_id, values)

    def get(self, control, motor_ids):
        """Send a read instruction and update memory

        :param control:    the control involved
        :param motor_ids:  ids of motors. If more than one, and the io supports
                           it, do a sync write.
        """
        assert len(motor_ids) > 0
        if len(motor_ids) == 1:
            self._send_read_packet(control, motor_ids[0])
        else:
            if self.sio.support_sync_read:
                self._send_sync_read_packet(control, motor_ids)
            else:
                for motor_id in motor_ids:
                    self._send_read_packet(control, motor_id)

    # MARK - Special cases

    def change_id(self, motor_id, new_motor_id):
        """
        Changes the id of the motor.

        Each motor must have a unique id on the bus.
        The range of possible ids is [0, 253].

        :param int motor_id: current motor id
        :param int new_motor_id: new motor id
        :raises: ValueError when the id is already taken
        """
        if motor_id != new_motor_id and self.ping(new_motor_id):
            raise ValueError('id %d already used' % (new_motor_id))

        self._send_write_packet(motor_id, 'ID', new_motor_id)

        mmem = self.motormems.pop(motor_id)
        mmem[pt.ID] = new_motor_id
        mmem.id = new_motor_id
        self.motormems[new_motor_id] = mmem

    def get_status_return_level(self, motor_id):
        """
        Returns the level of status return.

        Threre are three levels of status return:
            * 0 : no return packet
            * 1 : return status packet only for the read instruction
            * 2 : always return a status packet

        :param int motor_id: specified motor id [0-253]

        .. note:: if the EEPROM has been properly loaded, executing this
                   is a waste of a good serial packet.
        """

        try:
            status_return_level = self._send_read_packet(pt.STATUS_RETURN_LEVEL, motor_id)
            self.motormems[motor_id][pt.STATUS_RETURN_LEVEL] = status_return_level
        except TimeoutError as e:
            if self.ping(motor_id):
                self.motormems[motor_id][pt.STATUS_RETURN_LEVEL] = 0
            else:
                raise e

    def lock_eeprom(self, motor_id):
        """ Prevents the modification of the EEPROM area.

            .. warning:: Once the EEPROM is locked, you can't unlock it unless you
                         cycle the power.
        """
        self._send_write_packet(pt.LOCK, motor_id, (1,))
        self.motormems[motor_id][pt.LOCK.addr] = 1

    def unlock_eeprom(self, motor_id):
        """
            .. warning:: You can't unlock the EEPROM once it's locked.
        """
        raise DeprecationWarning('to unlock the eeprom, you should cycle power')
        #self._send_write_packet(pt.LOCK, motor_id, (0,))
        #self.motormems[motor_id][pt.LOCK.addr] = 0


    # MARK: - Low level communication

    def _send_packet(self, inst_packet, receive=True):
        """Send a packet and handle the (eventual) reception"""
        with self._lock:
            n = self.sio.write(str(inst_packet.data))
            if n != len(inst_packet):
                raise CommunicationError('Packet not correctly sent', packet, None)

            if receive:
                data = self.sio.read(packet.HEADER_SIZE)
                if len(data) < packet.HEADER_SIZE:
                    data += self.sio.read(packet.HEADER_SIZE-len(data))
                if len(data) == 0:
                    raise TimeoutError(inst_packet)

                try:
                    packet.check_header(inst_packet.motor_id, data)
                except AssertionError as e:
                    raise CommunicationError(e.args[0],
                                             inst_packet, bytearray(data))

                try:
                    data += self.sio.read(ord(data[3]))
                    status_packet = packet.StatusPacket(data)

                except packet.PacketError as e:
                    raise CommunicationError(e.msg, inst_packet,
                                             list(bytearray(data)))

                if status_packet.error != 0:
                    alarms = conv.raw2_alarm_names(status_packet.error)
                    if len(alarms):
                        raise MotorError(status_packet.motor_id, alarms)

                return status_packet

    def _update_memory(self, control, motor_id, values):
        """Update the memory of the motors"""
        offset = 0
        for size, value in zip(control.sizes, values):
            self.motormems[motor_id][control.addr+offset] = value
            offset += size
        self.motormems[motor_id].update() # could be more selective, but this would be useless optimization.

    def _send_read_packet(self, control, motor_id):
        """Send a read packet and update memory if successful."""
        if self.motormems[motor_id].status_return_level == 0:
            print(('warning, status_return_level of motor {} is at 0, '
                   'no reads possible').format(motor_id))

        else:
            read_packet = packet.InstructionPacket(motor_id, pt.READ_DATA, (control.addr, sum(control.sizes)))
            status_packet = self._send_packet(read_packet, receive=True)

            if status_packet:
                values = self._to_values(control, status_packet.params)
                self._update_memory(control, motor_id, values)
                return values

    def _send_sync_read_packet(self, control, motor_ids):
        raise NotImplementedError

    def _send_write_packet(self, control, motor_id, values):
        """Send a write packet and update memory optimistically if no error."""
        params = self._to_params(control, values)

        write_packet = packet.InstructionPacket(motor_id, pt.WRITE_DATA, [control.addr] + params)

        self._send_packet(write_packet, receive=self.motormems[motor_id].status_return_level == 2)
        self._update_memory(control, motor_id, values)

    def _send_sync_write_packet(self, control, motor_ids, valuess):
        """
        Parameters layout is (details: http://support.robotis.com/en/product/dynamixel/communication/dxl_instruction.htm):
        [start addr, length of data to write, id0, param0id0, param1id1, ...,
                                                 id1, param0id1, param2id2, ...]
        """
        params = itertools.chain.from_iterable((
                    [motor_id]+self._to_params(control, values)
                    for motor_id, values in zip(motor_ids, valuess)))

        sync_write_packet = packet.InstructionPacket(pt.BROADCAST, pt.SYNC_WRITE, [control.addr]+list(params))
        self._send_packet(sync_write_packet, receive=False)
        for motor_id, values in zip(motor_ids, valuess):
            self._update_memory(control, motor_id, values)

    # MARK : - Parameter encoding/decoding

    @staticmethod
    def _to_values(control, values):
        """
        Transform parameters of a status packet in one and two bytes values
        """
        assert sum(control.sizes) == len(values), "{} should have length {} but has {}".format(list(values), sum(control.sizes), len(values))
        itv = values.__iter__()
        return [itv.next() if s == 1 else (itv.next() + itv.next() << 8)
                for s in control.sizes]

    @staticmethod
    def _to_params(control, params):
        """
        Transform one bytes and two bytes values into parameters for
        an instruction packet
        """
        print(params)
        assert len(control.sizes) == len(params), "{} and {} don't have the same length".format(control.sizes, list(params))
        data = []
        for d, s in zip(params, control.sizes):
            if s == 1:
                data.append(d)
            elif s == 2:
                data.append(d % 255)
                data.append(d >> 8)
        return data

