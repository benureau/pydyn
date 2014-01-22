# -*- coding: utf-8 -*-

import time
import array
import serial
import threading

from pydyn.utils import flatten_list, reshape_list

from . import protocol
from . import packet
from . import limits
from . import memory

from . import alarms as conv # the only conversion needed in I/O

# MARK: - Byte conversions

def integer_to_two_bytes(value):
    return (int(value % 256), int(value >> 8))

def two_bytes_to_integer(value):
    return int(value[0] + (value[1] << 8))


class DynamixelIOSerial:
    """
        This class handles the low-level communication with robotis motors.

        Using a USB communication device such as USB2DYNAMIXEL or USB2AX,
        you can open serial communication with robotis motors (MX, RX, AX)
        using communication protocols TTL or RS485.

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
        are made through the motor interface.

        Also, this module does minimal checking about if values make sense or are legal.
        The Motor instance is responsible for that. It makes everything simpler here, and
        problems are catched faster by the Motor instances. Plus, if you really want to go
        crazy, you can.

        .. warning:: When accessing EEPROM registers the motor enters a "busy" mode and should not be accessed before about 100ms.

        """

    __open_ports = []

    ### Note to developpers : if you make change, make sure the update to mmem are
    ###                       done after the serial communication. That way, if the
    ###                       communication fails, our memory does not contain bogus
    ###                       data.

    def __init__(self, port, baudrate=1000000, timeout=1.0, blacklisted_alarms=(), **kwargs):
        """
            At instanciation, it opens the serial port and sets the communication parameters.

            .. warning:: The port can only be accessed by a single DynamixelIO instance.

            :param string port: the serial port to use (e.g. Unix (/dev/tty...), Windows (COM...)).
            :param int baudrate: default for new motors: 57600, for PyDyn motors: 1000000
            :param float timeout: read timeout in seconds
            :param blacklisted_alarms: list of blacklisted alarms that will not be triggered as :py:exc:`DynamixelMotorError`
            :type blacklisted_alarms: list of elements of :py:const:`~pydyn.dynamixel.protocol.DXL_ALARMS`

            :raises: IOError (when port is already used)

            """
        if port in self.__open_ports:
            raise IOError('Port already used (%s)!' % (port))

        self._timeout = timeout
        self._serial = serial.Serial(port, baudrate, timeout=timeout, stopbits=serial.STOPBITS_TWO)
        self.__open_ports.append(port)
        self.flush()

        self.blacklisted_alarms = blacklisted_alarms

        self._lock = threading.RLock()

        self.motormems = {}

    def close(self):
        if hasattr(self, '_serial'):
            self._lock.acquire()
            try:
                self.__open_ports.remove(self._serial.port)
                self._serial.close()
            except:
                pass

    def __del__(self):
        """ Automatically closes the serial communication on destruction. """
        self.close()

    def __repr__(self):
        return "<DXL IO: port='%s' baudrate=%d timeout=%.g>" % (self._serial.port,
                                                                self._serial.baudrate,
                                                                self._serial.timeout)


    def flush(self):
        """
            Flush the serial communication (both input and output).

            .. note:: You can use this method after a communication issue (such as a timeout) to refresh the communication bus.

            """
        self._serial.flushInput()
        self._serial.flushOutput()


    # MARK: - Motor general functions

    def ping(self, motor_id):
        """
            Pings the motor with the specified id.

            :param int motor_id: specified motor id [0-253]
            :return: bool
            :raises: ValueError if the motor id is out of the possible ids range.

            """
        if not (0 <= motor_id <= 253):
            raise ValueError('Motor id must be in [0, 253]!')

        ping_packet = packet.DynamixelPingPacket(motor_id)

        try:
            self._send_packet(ping_packet)
            return True

        except DynamixelTimeoutError:
            return False


    def scan(self, ids=xrange(254)):
        """
            Finds the ids of all the motors connected to the bus.

            :param list ids: the range of ids to search
            :return: list of ids found

            """
        return filter(self.ping, ids)


    def read(self, motor_id, address, size):
        """
            Read arbitrary data from a motor

            :param address  where to read data in the memory.
            :param size     how much from adress.
            :return: list of integers with asked size

            """
        ipacket = packet.DynamixelInstructionPacket(motor_id, 'READ_DATA', (address, size))
        status_packet = self._send_packet(ipacket)
        return status_packet.parameters


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
                mem.process_extra(raw_extra)

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

    def set(self, motor_id, control_name, value):
        """Send a write instruction to a motor regarding control_name"""

        self._send_write_packet(motor_id, control_name, value)

        self.motormems[motor_id][protocol.REG_ADDRESS(control_name)] = value
        self.motormems[motor_id].update() # could be more selective, but this would be useless optimization.

    def get(self, motor_id, control_name):
        """Send a read instruction to a motor regarding control_name"""

        value = self._send_read_packet(motor_id, control_name)

        self.motormems[motor_id][protocol.REG_ADDRESS(control_name)] = value
        self.motormems[motor_id].update() # could be more selective, but this would be useless optimization.


    # MARK Mode

    def change_mode(self, motor_id, mode):
        if mode == 'wheel':
            self.set_to_wheel_mode(motor_id)
        else:
            self.set_to_joint_mode(motor_id)

    def set_to_wheel_mode(self, motor_id):
        """
            Set the motor to wheel mode

            .. warning:: the speed unit changes when changing the mode. Adjust speed beforhand accordingly.

            """
        mmem = self.motormems[motor_id]
        if mmem.mode == 'wheel':
            return

        self._send_write_packet(motor_id, 'ANGLE_LIMITS', (0, 0))

        mmem.mode = 'wheel'
        mmem[protocol.DXL_CW_ANGLE_LIMIT]  = 0
        mmem[protocol.DXL_CCW_ANGLE_LIMIT] = 0

    def set_to_joint_mode(self, motor_id, angle_limits = None):
        """
            Set the motor to join mode

            :param angle_limits  (cw, ccw) desired angle limits.
                                 if not provided, default to maximum range.

            .. warning:: the speed unit changes when changing the mode. Adjust speed beforhand accordingly.

            """
        mmem = self.motormems[motor_id]
        if mmem.mode == 'joint':
            return

        if angle_limits is None:
            angle_limits = (0, limits.position_range[mmem.modelclass][1])

        self.set_angle_limits(motor_id, *angle_limits)

        mmem.mode = 'joint'
        mmem[protocol.DXL_CW_ANGLE_LIMIT]  = angle_limits[0]
        mmem[protocol.DXL_CCW_ANGLE_LIMIT] = angle_limits[1]


    # MARK: - Sync Read/Write

    def get_sync_positions(self, motor_ids):
        """
            Synchronizes the getting of positions in degrees of all motors specified.

            :param motor_ids: specified motor ids [0-253]
            :type motor_ids: list of ids

            .. warning:: This method only works with the USB2AX.

            """
        motor_positions = self._send_sync_read_packet(motor_ids, 'PRESENT_POSITION')

        for motor_id, pos in zip(motor_ids, motor_positions):
            self.motormems[motor_id][protocol.DXL_PRESENT_POSITION] = pos

    def set_sync_positions(self, id_pos_pairs):
        """
            Synchronizes the setting of the specified positions (in degrees) to the motors.

            :type id_pos_pairs: list of couple (motor id, position)

            """
        self._send_sync_write_packet('GOAL_POSITION', id_pos_pairs)

        for motor_id, pos in id_pos_pairs:
            self.motormems[motor_id][protocol.DXL_GOAL_POSITION] = pos

    set_sync_goal_position = set_sync_positions

    def get_sync_speeds(self, motor_ids):
        """
            Synchronizes the getting of speed in dps of all motors specified.

            :param motor_ids: specified motor ids [0-253]
            :type motor_ids: list of ids

            .. warning:: This method only works with the USB2AX.

            """
        motor_speed = self._send_sync_read_packet(motor_ids, 'PRESENT_SPEED')

        for motor_id, pos in zip(motor_ids, motor_positions):
            self.motormems[motor_id][protocol.DXL_PRESENT_SPEED] = pos

    def set_sync_speeds(self, id_speed_pairs):
        """
            Synchronizes the setting of the specified speeds (in dps) to the motors.

            :type id_speed_pairs: list of couple (motor id, speed)

            """

        # TODO : in Motor, verify that speed is positive in joint mode.


        self._send_sync_write_packet('MOVING_SPEED', id_speed_pairs)

        for motor_id, speed in id_speed_pairs:
            self.motormems[motor_id][protocol.DXL_MOVING_SPEED] = speed


    def get_sync_loads(self, motor_ids):
        """
            Synchronizes the getting of load in percentage of all motors specified.

            :param motor_ids: specified motor ids [0-253]
            :type motor_ids: list of ids

            .. warning:: This method only works with the USB2AX.

            """
        loads = self._send_sync_read_packet(motor_ids, 'PRESENT_LOAD')

        for motor_id, load in zip(motor_ids, loads):
            self.motormems[motor_id][protocol.DXL_PRESENT_LOAD] = load

    def set_sync_torque_limits(self, id_torque_pairs):
        """
            Synchronizes the setting of the specified torque limits to the motors.

            :type id_torque_pairs: list of couple (motor id, torque)

            """
        self._send_sync_write_packet('TORQUE_LIMIT', id_torque_pairs)

        for motor_id, torque in id_torque_pairs:
             self.motormems[motor_id][protocol.DXL_TORQUE_LIMIT] = torque


    def get_sync_positions_speeds_loads(self, motor_ids):
        """
            Synchronizes the getting of positions, speeds, load of all motors specified.

            :param motor_ids: specified motor ids [0-253]
            :type motor_ids: list of ids
            :return: list of (position, speed, load)

            .. warning:: This method only works with the USB2AX.

            """

        pos_speed_loads = self._send_sync_read_packet(motor_ids, 'PRESENT_POS_SPEED_LOAD')

        for motor_id, psl in zip(motor_ids, pos_speed_loads):
            pos, speed, load = psl
            mmem = self.motormems[motor_id]
            mmem[protocol.DXL_PRESENT_POSITION] = pos
            mmem[protocol.DXL_PRESENT_SPEED]    = speed
            mmem[protocol.DXL_PRESENT_LOAD]     = load


    def set_sync_positions_speeds_torque_limits(self, id_pos_speed_torque_tuples):
        """
            Synchronizes the setting of the specified positions, speeds and torque limits (in their respective units) to the motors.

            * The position is expressed in degrees.
            * The speed is expressed in dps (positive values correspond to clockwise).
            * The torque limit is expressed as a percentage of the maximum torque.

            :param id_pos_speed_torque_tuples: each value must be expressed in its own units.
            :type id_pos_speed_torque_tuples: list of (motor id, position, speed, torque)

            """

        self._send_sync_write_packet('GOAL_POS_SPEED_TORQUE', id_pos_speed_torque_tuples)

        for motor_id, pos, speed, torque in id_pos_speed_torque_tuples:
            mmem = self.motormems[motor_id]
            mmem[protocol.DXL_GOAL_POSITION] = pos
            mmem[protocol.DXL_MOVING_SPEED]  = speed
            mmem[protocol.DXL_TORQUE_LIMIT]  = torque

    def set_sync_speeds_torque_limits(self, id_speed_torque_tuples):
        """See doc for set_sync_positions_speeds_torque_limits, and remove position of it."""

        self._send_sync_write_packet('SPEED_TORQUE', id_speed_torque_tuples)

        for motor_id, speed, torque in id_speed_torque_tuples:
            mmem = self.motormems[motor_id]
            mmem[protocol.DXL_MOVING_SPEED]  = speed
            mmem[protocol.DXL_TORQUE_LIMIT]  = torque



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
        mmem[protocol.DXL_ID] = new_motor_id
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

            .. note:: if the EEPROM has been properly loaded, calling this function is a waste of a good serial packet.
            """

        try:
            status_return_level = self._send_read_packet(motor_id, 'STATUS_RETURN_LEVEL')
            self.motormems[motor_id][protocol.DXL_STATUS_RETURN_LEVEL] = status_return_level
        except DynamixelTimeoutError as e:
            if self.ping(motor_id):
                self._motor_return_level[motor_id] = 0
                self.motormems[motor_id][protocol.DXL_STATUS_RETURN_LEVEL] = status_return_level
            else:
                raise e

    def lock_eeprom(self, motor_id):
        """ Prevents the modification of the EEPROM area.

            .. warning:: Once the EEPROM is locked, you can't unlock it unless you
                         cycle the power.
        """
        self._send_write_packet(motor_id, 'LOCK', 1)

        self.motormems[motor_id][protocol.DXL_LOCK] = 1

    def unlock_eeprom(self, motor_id):
        """
            .. warning:: You can't unlock the EEPROM once it's locked.
        """
        raise DeprecationWarning('to unlock the eeprom, you should cycle power')
        self._send_write_packet(motor_id, 'LOCK', 0)

        self.motormems[motor_id][protocol.DXL_LOCK] = 0


    # MARK: - Low level communication

    def _send_packet(self, instruction_packet, receive_status_packet=True):
        """
            Sends a specified instruction packet to the serial port.

            If a response is required from the motors, it returns a status packet.
            : warn a status packet will not be returned for all sync write
            operation. All other operations will return a status packet that
            must be read from the serial in order to prevent leaving them
            on the serial buffer.

            Parameters
            ----------
            instruction_packet: packet.DynamixelInstructionPacket

            Returns
            -------
            status_packet: packet.DynamixelStatusPacket

            See Also
            --------
            packet.py

            """
        with self._lock:
            nbytes = self._serial.write(instruction_packet.to_bytes())
            if nbytes != len(instruction_packet):
                raise DynamixelCommunicationError('Packet not correctly sent',
                                                  instruction_packet,
                                                  None)

            if not receive_status_packet:
                return

            read_bytes = list(self._serial.read(packet.DynamixelPacketHeader.LENGTH))
            maxtries = int(0.05/max(self._timeout, 0.001))
            tries = 0
            while read_bytes == []:
                if tries >= maxtries:
                    raise DynamixelTimeoutError(instruction_packet)
                time.sleep(0.01)
                read_bytes = list(self._serial.read(packet.DynamixelPacketHeader.LENGTH))
                tries += 1

            try:
                header = packet.DynamixelPacketHeader.from_bytes(read_bytes)
                read_bytes += self._serial.read(header.packet_length)
                status_packet = packet.DynamixelStatusPacket.from_bytes(read_bytes)

            except packet.DynamixelInconsistentPacketError as e:
                raise DynamixelCommunicationError(e.message,
                                                  instruction_packet,
                                                  read_bytes)

            if status_packet.error != 0:
                alarms = conv.raw2_alarm_names(status_packet.error)
                alarms = filter(lambda a: a not in self.blacklisted_alarms, alarms)

                if len(alarms):
                    raise DynamixelMotorError(status_packet.motor_id, alarms)

            return status_packet


    def _send_read_packet(self, motor_id, control_name):
        """
            This reads the data returned by the status packet of the specified motor.

            Parameters
            ----------
            motor_id: int
            control_name: string
                a DXL_CONTROL key (see protocol.py)

            Returns
            -------
            status_packet (PARAM): data received inside the returned packet
                This is only the parameter set provided within the status
                packet sent from the motor.
            """
        # if self.motormems[motor_id].status_return_level == 0:
        #     self._send_write_packet(motor_id, 'STATUS_RETURN_LEVEL', 2)
        #     self.motormems[motor_id][protocol.DXL_STATUS_RETURN_LEVEL] = 2

        read_packet = packet.DynamixelReadDataPacket(motor_id, control_name)
        status_packet = self._send_packet(read_packet)

        if status_packet:
            return self._decode_data(status_packet.parameters,
                                     protocol.REG_SIZE(control_name))


    def _send_sync_read_packet(self, motor_ids, control_name):
        """
            This reads the data returned by the status packet of all
            the specified motors.

            Parameters
            ----------
            motor_ids: list
                list of ids of all the motors that return data
            control_name: string
                a DXL_CONTROL key (see protocol.py)

            Returns
            -------
            status_packet (PARAM): data received inside the returned packet
                This is only the parameter set provided within the status
                packet sent from the motors.

            """
        read_packet = packet.DynamixelSyncReadDataPacket(motor_ids, control_name)
        status_packet = self._send_packet(read_packet)

        answer = reshape_list(status_packet.parameters, protocol.REG_LENGTH(control_name))

        return map(lambda data: self._decode_data(data, protocol.REG_SIZE(control_name)),
                   answer)


    def _send_write_packet(self, motor_id, control_name, data):
        """
            This creates a write packet for the specified motor with the
            specified data.

            Parameters
            ----------
            motor_id: int
            control_name: string
                a DXL_CONTROL key (see protocol.py)
            data: int, tuple
                data to write to the motors
            """
        # if control_name != 'STATUS_RETURN_LEVEL':
        #     #self.motormems[motor_id][protocol.DXL_STATUS_RETURN_LEVEL] = 2
        #     self._send_write_packet(motor_id, 'STATUS_RETURN_LEVEL', 2)

        data = self._code_data(data, protocol.REG_SIZE(control_name))

        write_packet = packet.DynamixelWriteDataPacket(motor_id, control_name, data)

        receive_status_packet = True if self.motormems[motor_id].status_return_level == 2 else False
        self._send_packet(write_packet, receive_status_packet)

    def _send_sync_write_packet(self, control_name, data_tuples):
        """
            This creates and sends a sync write packet for specified motors
            given specified data.

            The data to be written is a list of tuples, where the tuples are
            in the following form:
                (MOTOR_ID, DATA1, DATA2,...)


            Parameters
            ----------
            data_tuple: list(tuple)
                data to write
            control_name: string
                a DXL_CONTROL key (see protocol.py)

            """
        code_func = lambda chunk: [chunk[0]] + self._code_data(chunk[1:], protocol.REG_SIZE(control_name))
        data = flatten_list(map(code_func, data_tuples))

        sync_write_packet = packet.DynamixelSyncWriteDataPacket(control_name, data)
        self._send_packet(sync_write_packet, receive_status_packet=False)

    # MARK : - Data coding/uncoding

    def _code_data(self, data, data_length):
        """
            This transforms the data into the correct format so that
            it can be added into a message packet

            Parameters
            ----------
            data: int, list(int)
                The data to be transformed
            data_length: int
                The number of registers used to code the data. For these motors
                the value can only be either 1 or 2.

            Returns
            -------
            list(data): list()
                the correctly formatted data

            """
        if data_length not in (1, 2):
            raise ValueError('Unsupported size of data (%d)' % (data_length))

        if not hasattr(data, '__len__'):
            data = [data]

        if data_length == 2:
            data = flatten_list(map(integer_to_two_bytes, data))

        return list(data)

    def _decode_data(self, data, data_length):
        """
            This transforms the data received in a packet into a useable format.

            Parameters
            ----------
            data: bytes, list(bytes)
                The data to be transformed
            data_length: int
                The number of registers used to code the data. For these motors
                the value can only be either 1 or 2.

            Returns
            -------
            list(data): list()
                the correctly formatted data

            """
        if data_length not in (1, 2):
            raise ValueError('Unsupported size of data (%d)' % (data_length))

        if data_length == 2:
            data = map(two_bytes_to_integer, reshape_list(data, 2))
        return data if len(data) > 1 else data[0]


# MARK: - Dxl Error

class DynamixelCommunicationError(Exception):
    def __init__(self, message, instruction_packet, response):
        self.message = message
        self.instruction_packet = instruction_packet
        self.response = map(ord, response) if response else None

    def __str__(self):
        return '%s (instruction packet: %s, status packet: %s)' \
            % (self.message, self.instruction_packet, self.response)

class DynamixelTimeoutError(DynamixelCommunicationError):
    def __init__(self, instruction_packet):
        DynamixelCommunicationError.__init__(self, 'Timeout', instruction_packet, None)

class DynamixelMotorError(Exception):
    def __init__(self, motor_id, alarms):
        self.motor_id = motor_id
        self.alarms = alarms

    def __str__(self):
        return 'Motor %d triggered alarm%s: %s' % (self.motor_id,
                                                   's' if len(self.alarms) > 1 else '',
                                                   self.alarms if len(self.alarms) > 1 else self.alarms[0])

class DynamixelUnsupportedFunctionForMotorError(Exception):
    def __init__(self, func, motor_id, motor_model):
        self.func = func
        self.motor_id = motor_id
        self.motor_model = motor_model

    def __str__(self):
        return 'Unsupported function (%s) for motor (%d: %s)' % (self.func.__name__, self.motor_id, self.motor_model)
