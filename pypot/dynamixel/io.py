# -*- coding: utf-8 -*-

import array
import serial
import threading

from pypot.utils import flatten_list, reshape_list

import protocol
import packet
import limits
import memory

# MARK: - Byte conversions

def integer_to_two_bytes(value):
    return (int(value % 256), int(value >> 8))

def two_bytes_to_integer(value):
    return int(value[0] + (value[1] << 8))
    

class DynamixelIO:
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


    def __init__(self, port, baudrate=1000000, timeout=1.0, blacklisted_alarms=()):
        """
            At instanciation, it opens the serial port and sets the communication parameters.

            .. warning:: The port can only be accessed by a single DynamixelIO instance.

            :param string port: the serial port to use (e.g. Unix (/dev/tty...), Windows (COM...)).
            :param int baudrate: default for new motors: 57600, for PyPot motors: 1000000
            :param float timeout: read timeout in seconds
            :param blacklisted_alarms: list of blacklisted alarms that will not be triggered as :py:exc:`DynamixelMotorError`
            :type blacklisted_alarms: list of elements of :py:const:`~pypot.dynamixel.protocol.DXL_ALARMS`

            :raises: IOError (when port is already used)

            """
        if port in self.__open_ports:
            raise IOError('Port already used (%s)!' % (port))

        self._serial = serial.Serial(port, baudrate, timeout=timeout)
        self.__open_ports.append(port)
        self.flush_serial_communication()

        self.blacklisted_alarms = blacklisted_alarms

        self._lock = threading.RLock()

        self.motormems = {}

    def __del__(self):
        """ Automatically closes the serial communication on destruction. """
        if hasattr(self, '_serial'):
            self._lock.acquire()
            self.__open_ports.remove(self._serial.port)
            self._serial.close()

    def __repr__(self):
        return "<DXL IO: port='%s' baudrate=%d timeout=%.g>" % (self._serial.port,
                                                                self._serial.baudrate,
                                                                self._serial.timeout)


    def flush_serial_communication(self):
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

    def create(self, ids, cache_ram = True):
        """
            Load the motor memory.
    
            :param list ids, the ids to create.
            :return: instances of DynamixelMemory
    
            .. warning:: we assume ids have been checked by a previous ping.
            .. note:: if a memory already exist, it is recreated anyway.
            """
        created = []
        for motor_id in ids:
            raw_eeprom = self.read(motor_id, 0, 24)
            m = memory.DynamixelMemory(raw_eeprom)
            if cache_ram:
                raw_ram = self.read(motor_id, 24, 26)
                if m.modelclass == 'MX':
                    raw_ram += self.read(motor_id, 68, 2)
                m.cache_ram(raw_ram)
            created.append(m)
            self.motormems[motor_id] = m
    
        return created


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


    # MARK - EEPROM Access
    # Those values should not be modified too often !!!

    def get_model(self, motor_id):
        """
            Finds the model name of the robotis motor.

            :param int motor_id: specified motor id [0-253]
            :raises: DynamixelUnsupportedMotorError if the motor model is currently unsupported.

            .. note:: if the EEPROM has been properly loaded, calling this function is a waste
                      of a good serial packet.
            """

        model_number = self._send_read_packet(motor_id, 'MODEL_NUMBER')

        mmem = self.motormems[motor_id]
        mmem[protocol.DXL_MODEL_NUMBER] = model_number
        mmem.update()

    def change_id(self, motor_id, new_motor_id):
        """
            Changes the id of the motor.

            Each motor must have a unique id on the bus.
            The range of possible ids is [0, 253].

            :param int motor_id: current motor id
            :param int new_motor_id: new motor id
            :raises: ValueError when the id is already taken

            """
        if self.ping(new_motor_id):
            raise ValueError('id %d already used' % (new_motor_id))

        self._send_write_packet(motor_id, 'ID', new_motor_id)

        mmem = self.motormems.pop(motor_id)
        mmem[protocol.DXL_ID] = new_motor_id
        mmem.id = new_motor_id
        self.motormems[new_motor_id] = mmem

    def set_baudrate(self, motor_id, baudrate):
        """
            Changes the baudrate of a specified motor.

            :param int motor_id: specified motor id [0-253]
            :param int baudrate: for a list of possible values see http://support.robotis.com/en/product/dxl_main.htm

            .. warning:: The baudrate of the motor must match the baudrate of the port. When the baurate is changed for the motor, it will be necessary to open a new port with the new baurate to re-establish communication.

            """
        self._send_write_packet(motor_id, 'BAUDRATE', baudrate)

        self.motormems[motor_id][protocol.DXL_BAUDRATE] = baudrate


    def get_return_delay_time(self, motor_id):
        """
            Returns the current delay time.

            :param int motor_id: specified motor id [0-253]
            :return: return delay time in µs

            .. note:: if the EEPROM has been properly loaded, calling this function is a waste of a good serial packet.
            """
        rdt = self._send_read_packet(motor_id, 'RETURN_DELAY_TIME')

        self.motormems[motor_id][protocol.DXL_RETURN_DELAY_TIME] = rdt


    def set_return_delay_time(self, motor_id, return_delay_time):
        """
            Sets a new return delay time in µs.

            :param int motor_id: specified motor id [0-253]
            :param int return_delay_time: new return delay time in µs [0-508]
            :raises: ValueError if the return delay time is out of the specified range.

            """
        self._send_write_packet(motor_id, 'RETURN_DELAY_TIME', return_delay_time)

        self.motormems[motor_id][protocol.DXL_RETURN_DELAY_TIME] = return_delay_time


    def get_angle_limits(self, motor_id):
        """
            Gets the lower and upper angle limits of the motor.

            .. note:: If you try to set a position to a motor outside of these angle limits, the motor will stop at the limit and an alarm will be triggered.

            :param int motor_id: specified motor id [0-253]
            :return: (lower_limit, upper_limit) in degrees

            .. note:: if the EEPROM has been properly loaded, calling this function is a waste of a good serial packet.

            .. warning:: If you try to set a position to a motor outside of these angle limits, the motor will stop at the limit and an alarm will be triggered.

            """
        angle_limits = self._send_read_packet(motor_id, 'ANGLE_LIMITS')

        mmem = self.motormems[motor_id]
        mmem[protocol.DXL_CW_ANGLE_LIMIT] = angle_limits[0]
        mmem[protocol.DXL_CCW_ANGLE_LIMIT] = angle_limits[1]
        mmem.update()

    def set_angle_limits(self, motor_id,
                         lower_limit, upper_limit):
        """
            Sets the lower and upper angle limits of the motor.

            .. note:: If you try to set a position to a motor outside of these angle limits, the motor will stop at the limit and an alarm will be triggered.

            :param int motor_id: specified motor id [0-253]
            :param float lower_limit: the lower angle limit
            :param float upper_limit: the upper angle limit
            :raises: ValueError if the lower limit is greater or equal than the upper limit

            .. note:: The angle limits must belong to the hardware limits of the motor (e.g. (-150, 150) for an AX-12 motor or (-180, 180) for a MX-28 motor).

            """
        # TODO
        # if self.get_mode(motor_id) == 'WHEEL':
        #     raise ValueError('Cannot change the angle limits in wheel mode (motor %d)' % (motor_id))
        #
        # if lower_limit >= upper_limit:
        #     raise ValueError('The lower limit (%d) must be less than the upper limit (%d).'
        #                      % (lower_limit, upper_limit))

        self._send_write_packet(motor_id,
                                'ANGLE_LIMITS',
                                (lower_limit, upper_limit))

        mmem = self.motormems[motor_id]
        mmem[protocol.DXL_CW_ANGLE_LIMIT]  = lower_limit
        mmem[protocol.DXL_CCW_ANGLE_LIMIT] = upper_limit
        mmem.update()

    def get_limit_temperature(self, motor_id):
        """
            Returns the motor highest limit of operating temperature.

            If the internal temperature of a motor exceeds this limit, it sets
            the Over Heating Error Bit of Status Packet to True and triggers an
            alarm (LED and shutdown).

            :param int motor_id: specified motor id [0-253]
            :return: limit of operating temperature in celsius

            .. note:: if the EEPROM has been properly loaded, calling this function is a waste of a good serial packet.

            """
        hlt = self._send_read_packet(motor_id, 'HIGHEST_LIMIT_TEMPERATURE')

        mmem[protocol.DXL_HIGHEST_LIMIT_TEMPERATURE] = hlt


    def set_limit_temperature(self, motor_id, limit_temperature):
        """
            Sets the highest limit of operating temperature.

            :param int motor_id: specified motor id [0-253]
            :param int limit_temperature: new limit of operating temperature [10-99]
            :raises: ValueError if the new temperature is outside of the allowed range

            .. warning:: Do not set the temperature lower/higher than the default value. Using the motor when the temperature is high may and can cause damage.

            """
        # if not (10 <= limit_temperature <= 99):
        #     raise ValueError('The temperature limit must be in [10,99]')

        self._send_write_packet(motor_id, 'HIGHEST_LIMIT_TEMPERATURE', limit_temperature)

        mmem[protocol.DXL_HIGHEST_LIMIT_TEMPERATURE] = limit_temperature


    def get_voltage_limits(self, motor_id):
        """
            Returns the operation range of voltage (V).

            If present voltage is out of the range, Voltage Range Error
            of Status Packet is returned as '1' and Alarm is triggered as set
            in the Alarm LED and Alarm Shutdown.

            :param int motor_id: specified motor id [0-253]

            .. note:: if the EEPROM has been properly loaded, calling this function is a waste of a good serial packet.

            """
        vl = self._send_read_packet(motor_id, 'VOLTAGE_LIMITS')

        mmem = self.motormems[motor_id]
        mmem[protocol.DXL_LOWEST_LIMIT_VOLTAGE] = vl[0]
        mmem[protocol.DXL_HIGHEST_LIMIT_VOLTAGE] = vl[1]

    def set_voltage_limits(self, motor_id,
                           lowest_limit_voltage, highest_limit_voltage):
        """
            Sets the operation voltage range for the motor.

            If Present Voltage is out of the range, Voltage Range Error
            of Status Packet is returned as '1' and Alarm is triggered as set
            in the Alarm LED and Alarm Shutdown.

            :param int motor_id: specified motor id [0-253]
            :param float lowest_limit_voltage: lowest operating voltage limit [5-25]
            :param float highest_limit_voltage: highest operating voltage limit [5-25]
            :raises: ValueError if the limit are out of range or if the lowest limit is greater than the highest limit

            """
        # TODO
        # if not (5 <= lowest_limit_voltage <= 25) or not (5 <= highest_limit_voltage <= 25):
        #     raise ValueError('The lowest limit temperature must be in [5, 25].')
        # if (highest_limit_voltage <= lowest_limit_voltage):
        #     raise ValueError('The highest limit voltage must be superior than the lowest limit voltage.')

        self._send_write_packet(motor_id,
                                'VOLTAGE_LIMITS',
                                (lowest_limit_voltage, highest_limit_voltage))

        mmem = self.motormems[motor_id]
        mmem[protocol.DXL_LOWEST_LIMIT_VOLTAGE] = lowest_limit_voltage
        mmem[protocol.DXL_HIGHEST_LIMIT_VOLTAGE] = highest_limit_voltage


    def get_max_torque(self, motor_id):
        """
            Returns the maximum output torque avaible in percent.

            .. note:: if the EEPROM has been properly loaded, calling this function is a waste of a good serial packet.

            """
        max_torque = self._send_read_packet(motor_id, 'MAX_TORQUE')

        self.motormems[motor_id][protocol.DXL_MAX_TORQUE] = max_torque


    def set_max_torque(self, motor_id, max_torque):
        """ Sets the maximum output torque avaible in percent. """
        # if not (0 <= max_torque <= 100):
        #     raise ValueError('The maximum torque must be in [0, 100].')

        self._send_write_packet(motor_id, 'MAX_TORQUE', max_torque)

        self.motormems[motor_id][protocol.DXL_MAX_TORQUE] = max_torque


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


    def set_status_return_level(self, motor_id, status_return_level):
        """ Sets the level of status return.

            Threre are three levels of status return:
                * 0 : no return packet
                * 1 : return status packet only for the read instruction
                * 2 : always return a status packet

            :param int motor_id: specified motor id [0-253]
            :param int status_return_level: level of status return
            """
        # TODO
        # if status_return_level not in (0, 1, 2):
        #     raise ValueError('Status level must be one of the following (0, 1, 2)')

        self._send_write_packet(motor_id, 'STATUS_RETURN_LEVEL', status_return_level)
        self.motormems[motor_id][protocol.DXL_STATUS_RETURN_LEVEL] = status_return_level


    def get_alarm_led(self, motor_id):
        """
            .. note:: if the EEPROM has been properly loaded, calling this function is a waste of a good serial packet.
            """
        alarm_led = self._send_read_packet(motor_id, 'ALARM_LED')

        self.motormems[motor_id][protocol.DXL_ALARM_LED] = alarm_led

    def set_alarm_led(self, motor_id, alarm_led):
        self._send_write_packet(motor_id, 'ALARM_LED', alarm_led)

        self.motormems[motor_id][protocol.DXL_ALARM_LED] = alarm_led


    def get_alarm_shutdown(self, motor_id):
        """
            .. note:: if the EEPROM has been properly loaded, calling this function is a waste of a good serial packet.
            """
        alarm_shutdown = self._send_read_packet(motor_id, 'ALARM_SHUTDOWN')

        self.motormems[motor_id][protocol.DXL_ALARM_SHUTDOWN] = alarm_shutdown

    def set_alarm_shutdown(self, motor_id, alarm_shutdown):
        self._send_write_packet(motor_id, 'ALARM_SHUTDOWN', alarm_shutdown)

        self.motormems[motor_id][protocol.DXL_ALARM_SHUTDOWN] = alarm_shutdown



    # MARK: - Dxl Motor RAM Access

    def get_torque_enable(self, motor_id):
        """
            Check if the torque is enabled for the specified motor.

            .. note:: The torque will automatically be enabled when setting a position.

            """
        torque_enable = self._send_read_packet(motor_id, 'TORQUE_ENABLE')

        self.motormems[motor_id][protocol.DXL_TORQUE_ENABLE] = torque_enable

    def set_torque_enable(self, motor_id, torque_enable):
        self._send_write_packet(motor_id, 'TORQUE_ENABLE', torque_enable)

        self.motormems[motor_id][protocol.DXL_TORQUE_ENABLE] = torque_enable

    def get_led(self, motor_id):
        """
            Checks if the led is on for the specified motor.

            """
        led = self._send_read_packet(motor_id, 'LED')
        self.motormems[motor_id][protocol.DXL_LED] = led

    def set_led(self, motor_id, on):

        self._send_write_packet(motor_id, 'LED', int(on))

        self.motormems[motor_id][protocol.DXL_TORQUE_ENABLE] = torque_enable


    def get_pid_gains(self, motor_id):
        """
            Gets the PID gains for the specified motor.

                * The P gain refers to the value of proportional band.
                * The I gain refers to the value of integral action.
                * The D gain refers to the value of derivative action.

            Gains values are in between [0-254].

            .. note:: This method only exists for MX motors. For other kinds of motors, see :py:meth:`get_compliance_margins` and :py:meth:`get_compliance_slopes`

            .. note:: As this value does not change unless the user modifies it, it is usually safe to use the cached value instead of reading the real one.

            """
        mmem = self.motormems[motor_id]

        if not mmem.modelclass == 'MX':
            raise DynamixelUnsupportedFunctionForMotorError(self.get_pid_gains,
                                                            motor_id,
                                                            mmem.model)

        d_gain, i_gain, p_gain = self._send_read_packet(motor_id, 'GAINS')

        mmem[protocol.DXL_P_GAIN] = p_gain
        mmem[protocol.DXL_I_GAIN] = i_gain
        mmem[protocol.DXL_D_GAIN] = d_gain

    def set_pid_gains(self, motor_id,
                      d_gain, i_gain, p_gain):
        """
            Sets the PID gains for the specified motor.

            :param int motor_id: specified motor id [0-253]
            :param int D_gain: refers to the value of derivative action [0-254]
            :param int I_gain: refers to the value of integral action [0-254]
            :param int P_gain: refers to the value of proportional band [0-254]

            .. note:: This method only exists for MX motors. For other kinds of motors, see :py:meth:`set_compliance_margins` and :py:meth:`set_compliance_slopes`

            """
        mmem = self.motormems[motor_id]

        if not mmem.modelclass == 'MX':
            raise DynamixelUnsupportedFunctionForMotorError(self.get_pid_gains,
                                                            motor_id,
                                                            mmem.model)

        self._send_write_packet(motor_id,
                                'GAINS',
                                (d_gain, i_gain, p_gain))

        mmem[protocol.DXL_P_GAIN] = p_gain
        mmem[protocol.DXL_I_GAIN] = i_gain
        mmem[protocol.DXL_D_GAIN] = d_gain


    def get_compliance_margins(self, motor_id):
        """
            Gets the compliance margins of the specified motor.

            The compliance margin exists in each direction (CW and CCW) and
            means the error between goal position and present position.
            The greater the value, the more difference occurs.

            :param int motor_id: specified motor id [0-253]
            :return: (cw margin, ccw margin)

            .. note:: For MX motors, this method has been replaced by :py:meth:`get_pid_gains`.

            .. note:: As this value does not change unless the user modifies it, it is usually safe to use the cached value instead of reading the real one.

            """
        mmem = self.motormems[motor_id]

        if not (mmem.modelclass == 'AX' or mmem.modelclass == 'RX'):
            raise DynamixelUnsupportedFunctionForMotorError(self.get_compliance_margins,
                                                            motor_id,
                                                            mmem.model)

        compliance_margins = self._send_read_packet(motor_id, 'COMPLIANCE_MARGINS')
        mmem[protocol.DXL_CW_COMPLIANCE_MARGIN] = compliance_margins[0]
        mmem[protocol.DXL_CCW_COMPLIANCE_MARGIN] = compliance_margins[1]

    def set_compliance_margins(self, motor_id,
                               clockwise_margin, counterclockwise_margin):
        """
            Sets new compliance margins for the specified motors.

            The compliance margin exists in each direction (CW and CCW) and
            means the error between goal position and present position.
            The greater the value, the more difference occurs.

            :param int clockwise_margin: clockwise margin [0-255]
            :param int counterclockwise_margin: counter clockwise margin [0-255]

            .. note:: For MX motors, this method has been replaced by :py:meth:`set_pid_gains`.

            """
        mmem = self.motormems[motor_id]

        if not (mmem.modelclass == 'AX' or mmem.modelclass == 'RX'):
            raise DynamixelUnsupportedFunctionForMotorError(self.get_compliance_margins,
                                                            motor_id,
                                                            mmem.model)

        self._send_write_packet(motor_id, 'COMPLIANCE_MARGINS',
                                (clockwise_margin, counterclockwise_margin))

        mmem[protocol.DXL_CW_COMPLIANCE_MARGIN] = clockwise_margin
        mmem[protocol.DXL_CCW_COMPLIANCE_MARGIN] = counterclockwise_margin

    def get_compliance_slopes(self, motor_id):
        """
            Gets the compliance slopes of the specified motor.

            The compliance slope exists in each direction (CW and CCW) and
            sets the level of torque near the goal position.
            The higher the value, the more flexibility is obtained.

            :param int motor_id: specified motor id [0-253]
            :return: (cw slope, ccw slope)

            .. note:: For MX motors, this method has been replaced by :py:meth:`get_pid_gains`.

            .. note:: As this value does not change unless the user modifies it, it is usually safe to use the cached value instead of reading the real one.

            """
        mmem = self.motormems[motor_id]

        if not (mmem.modelclass == 'AX' or mmem.modelclass == 'RX'):
            raise DynamixelUnsupportedFunctionForMotorError(self.get_compliance_margins,
                                                            motor_id,
                                                            mmem.model)

        clockwise_slope, counterclockwise_slope = self._send_read_packet(motor_id, 'COMPLIANCE_SLOPES')

        mmem[protocol.DXL_CW_COMPLIANCE_SLOPE] = clockwise_margin
        mmem[protocol.DXL_CCW_COMPLIANCE_SLOPE] = counterclockwise_margin

    def set_compliance_slopes(self, motor_id,
                              clockwise_slope, counterclockwise_slope):
        """
            Sets new compliance slopes for the specified motors.

            The compliance slope exists in each direction (CW and CCW) and
            sets the level of torque near the goal position.
            The higher the value, the more flexibility is obtained.

            :param int motor_id: specified motor id [0-253]
            :param int clockwise_slope: clockwise slope [0-255]
            :param int counterclockwise_slope: counter clockwise slope [0-255]

            .. note:: For MX motors, this method has been replaced by :py:meth:`set_pid_gains`.

            """
        mmem = self.motormems[motor_id]

        if not (mmem.modelclass == 'AX' or mmem.modelclass == 'RX'):
            raise DynamixelUnsupportedFunctionForMotorError(self.get_compliance_margins,
                                                            motor_id,
                                                            mmem.model)

        self._send_write_packet(motor_id, 'COMPLIANCE_SLOPES',
                                (clockwise_slope, counterclockwise_slope))

        mmem[protocol.DXL_CW_COMPLIANCE_SLOPE] = clockwise_slope
        mmem[protocol.DXL_CCW_COMPLIANCE_SLOPE] = counterclockwise_slope


    def get_current_position(self, motor_id):
        """ Get the current position of the specified motor. """
        pos = self._send_read_packet(motor_id, 'PRESENT_POSITION')

        self.motormems[motor_id][protocol.DXL_PRESENT_POSITION] = pos

    def get_goal_position(self, motor_id):
        """ Get the goal position of the specified motor. """
        pos = self._send_read_packet(motor_id, 'GOAL_POSITION')

        self.motormems[motor_id][protocol.DXL_GOAL_POSITION] = pos

    get_position = get_current_position

    def set_goal_position(self, motor_id, position):
        """ Sets the position of the specified motor. """
        self._send_write_packet(motor_id, 'GOAL_POSITION', position)

        self.motormems[motor_id][protocol.DXL_GOAL_POSITION] = pos

    set_position = set_goal_position

    def get_speed(self, motor_id):
        """ Returns the speed (positive values correspond to clockwise) of the specified motor. """
        speed = self._send_read_packet(motor_id, 'PRESENT_SPEED')

        self.motormems[motor_id][protocol.DXL_PRESENT_SPEED] = speed

    def set_speed(self, motor_id, speed):
        """ Sets the speed (positive values correspond to clockwise) of the specified motor. """
        # TODO
        # if self.get_mode(motor_id) == 'JOINT' and speed < 0:
        #     raise ValueError('Speed must always be positive in joint mode (motor %d)' % (motor_id))

        self._send_write_packet(motor_id, 'MOVING_SPEED', speed)

        self.motormems[motor_id][protocol.DXL_PRESENT_SPEED] = speed


    def get_torque_limit(self, motor_id):
        """
        .. note:: As this value does not change unless the user modifies it, it is usually
                  safe to use the cached value instead of reading the real one.

        """
        torque_limit = self._send_read_packet(motor_id, 'TORQUE_LIMIT')

        self.motormems[motor_id][protocol.DXL_TORQUE_LIMIT] = torque_limit

    def set_torque_limit(self, motor_id, torque_limit):
        self._send_write_packet(motor_id, 'TORQUE_LIMIT', torque_limit)

        self.motormems[motor_id][protocol.DXL_TORQUE_LIMIT] = torque_limit


    def get_load(self, motor_id):
        """ Get the internal load in percent of the specified motor. """
        self._send_read_packet(motor_id, 'PRESENT_LOAD')

        self.motormems[motor_id][protocol.DXL_PRESENT_LOAD]     = load

    def get_position_speed_load(self, motor_id):
        """ Get the position, speed and internal load of the specified motor """
        pos, speed, load = self._send_read_packet(motor_id, 'PRESENT_POS_SPEED_LOAD')

        mmem = self.motormems[motor_id]
        mmem[protocol.DXL_PRESENT_POSITION] = pos
        mmem[protocol.DXL_PRESENT_SPEED]    = speed
        mmem[protocol.DXL_PRESENT_LOAD]     = load


    def get_voltage(self, motor_id):
        """ Get the current voltage supplied (in Volt). """
        voltage = self._send_read_packet(motor_id, 'PRESENT_VOLTAGE')

        self.motormems[motor_id][protocol.DXL_PRESENT_VOLTAGE] = voltage


    def get_temperature(self, motor_id):
        """ Returns the internal temperature of the specified motor (in Celsius). """
        temp = self._send_read_packet(motor_id, 'PRESENT_TEMPERATURE')

        self.motormems[motor_id][protocol.DXL_PRESENT_TEMPERATURE] = temp

    def get_registred(self, motor_id): # TODO: ca fait quoi ?
        registered = self._send_read_packet(motor_id, 'REGISTERED')

        self.motormems[motor_id][protocol.DXL_REGISTERED] = moving

    def get_moving(self, motor_id):
        """ Checks if the motor is moving (whether goal position has been reached). """
        moving = self._send_read_packet(motor_id, 'MOVING')

        self.motormems[motor_id][protocol.DXL_MOVING] = moving

    def get_lock(self, motor_id):
        """ Checks if the EEPROM area can be modified. """
        lock = self._send_read_packet(motor_id, 'LOCK')

        self.motormems[motor_id][protocol.DXL_LOCK] = lock

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

    def get_punch(self, motor_id):
        punch = self._send_read_packet(motor_id, 'PUNCH')

        self.motormems[motor_id][protocol.DXL_PUNCH] = punch


    def set_punch(self, motor_id, punch):
        self._send_write_packet(motor_id, 'PUNCH', punch)

        self.motormems[motor_id][protocol.DXL_PUNCH] = punch

    # def get_sensed_current(self, motor_id):
    #     motor_model = self._lazy_get_model(motor_id)
    #     if not motor_model.startswith('EX'):
    #         raise DynamixelUnsupportedFunctionForMotorError(self.get_sensed_current,
    #                                                         motor_id,
    #                                                         motor_model)
    #     return self._send_read_packet(motor_id, 'SENSED_CURRENT')

    # def get_current(self, motor_id):
    #     motor_model = self._lazy_get_model(motor_id)
    #     if not motor_model in ('MX-64', 'MX-106'):
    #         raise DynamixelUnsupportedFunctionForMotorError(self.get_current,
    #                                                         motor_id,
    #                                                         motor_model)
    #     return self._send_read_packet(motor_id, 'CURRENT')

    # def set_current(self, motor_id, current):
    #     motor_model = self._lazy_get_model(motor_id)
    #     if not motor_model in ('MX-64', 'MX-106'):
    #         raise DynamixelUnsupportedFunctionForMotorError(self.get_current,
    #                                                         motor_id,
    #                                                         motor_model)
    #     self._send_write_packet(motor_id, 'CURRENT', current)


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
            if not read_bytes:
                raise DynamixelTimeoutError(instruction_packet)

            try:
                header = packet.DynamixelPacketHeader.from_bytes(read_bytes)
                read_bytes += self._serial.read(header.packet_length)
                status_packet = packet.DynamixelStatusPacket.from_bytes(read_bytes)

            except packet.DynamixelInconsistentPacketError as e:
                raise DynamixelCommunicationError(e.message,
                                                  instruction_packet,
                                                  read_bytes)

            if status_packet.error != 0:
                alarms = byte_to_alarms(status_packet.error)
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
        if self.motormems[motor_id].status_return_level == 0:
            raise IOError('Try to get a value from motor with a level of status return of 0')

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
