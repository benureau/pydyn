# -*- coding: utf-8 -*-

import array
import threading
import math

import pyvrep

from . import protocol
from . import memory
from . import conversions

default_eeprom = [
    44, 39, # Model = 10028
    24,     # Firware
    None,   ## ID
    1,      # Baudrate
    0,      # Return Delay Time
    0, 0,   # CW angles            # None ?
    255, 3,                        # None ?
    0,      ###
    80,     # High limit temp
    60,     # Low limit voltage
    240,    # High limit voltage
    255, 3, # Max Torque
    2,      # Status return level
    36,     # Alarm LED
    36,     # Alarm Shutdown
    ]

# A VRep motor is immobile, and its present position is also its target position.
default_ram = [
    1,          # Torque enable
    0,          # Led
    0,          # D gain
    0,          # I gain
    32,         # P gain
    0,          ###
    0, 0,       ## Goal pos
    0, 0,       ## Moving speed
    255, 3,     # Torque limit
    0, 0,       ## Present speed
    0, 0,       ## Present velocity
    0, 0,       ## Present load
    160,        # Present voltage
    40,         # Present temperature
    0,          # Registered
    0,          ###
    0,          # Moving
    0,          # Lock
    0, 0,       # Punch
    ]

MAXTORQUE = {'VX-28' : 28.3, 'VX-64': 64}

class DynamixelIOVRep(object):
    """
        This class emulate a low-level communication with the joints of a VRep instance.

        Users can use high-level access such as position, load, torque to control
        the motors.

        In this modules, all values are raw, integer values. Conversion to human values
        are made through the motor interface.
    """

    __open_ports = []
    __sims       = []

    def __init__(self, port, ip = '127.0.0.1', **kwargs):
        """
            At instanciation, it opens the serial port and sets the communication parameters.

            .. warning:: The port can only be accessed by a single DynamixelIO instance.

            :param string port: network ip to use (default localhost).
            :param string port: network port to use.

            :raises: IOError (when port is already used)

            """
        if port in self.__open_ports:
            raise IOError('Port already used (%s)!' % (port))

        self.sim = pyvrep.PyVrep()

        self.port = port
        self.ip   = ip
        self.__open_ports.append(port)
        self.__sims.append(self.sim)

        if self.sim.connectTo(self.ip, self.port) == -1:
            raise IOError('Connection to VREP instance {}:{} unsuccessful', self.ip, self.port)

        # stopping simulation if it is on
        if (self.sim.simGetSimulationState() != 0):
            self.sim.simStopSimulation()

        self.sim.registerBackgroundFunction("simGetSimulationTime", [])

        self.id2handle = {}
        self.handle2id = {}
        self.handles   = []
        self.created_ids = [] # list of created motors
        self.motormems = {}

        self.vx28 = set()
        self.vx64 = set()


    def close(self):
        self.sim.disconnect()

    def __del__(self):
        """ Automatically closes the VRep communication on destruction. """
        self.close()

    def __repr__(self):
        return "<DXL IO VREP: ip='%s' port='%s'>" % (self.ip, self.port)

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

        handle28 = self.sim.simGetObjectHandle('vx28_{}'.format(motor_id))
        handle64 = self.sim.simGetObjectHandle('vx64_{}'.format(motor_id))

        if handle28 != -1:
            self.handle2id[handle28] = motor_id
            self.id2handle[motor_id] = handle28
            self.vx28.add(motor_id)
            return True
        elif handle64 != -1:
            self.handle2id[handle64] = motor_id
            self.id2handle[motor_id] = handle64
            self.vx64.add(motor_id)
            return True
        else:
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
        pass

    def create(self, motor_ids):
        """
            Load the motor memory.

            :param list ids, the ids to create.
            :return: instances of DynamixelMemory

            .. warning:: we assume ids have been checked by a previous ping.
            .. note:: if a memory already exist, it is recreated anyway.
            """
        self._set_background_fun(motor_ids)

        mmems = []

        for motor_id in motor_ids:
            handle = self.id2handle[motor_id]
            self.created_ids.append(motor_id)

            # copy ram, eeprom
            raw_eeprom = default_eeprom[:]
            raw_ram    = default_ram[:]

            if motor_id in self.vx64:
                raw_eeprom[protocol.REG_ADDRESS("MODEL_NUMBER")] = 80
            raw_eeprom[protocol.REG_ADDRESS("ID")] = motor_id
            mmem = memory.DynamixelMemory(raw_eeprom, raw_ram)
            self.motormems[motor_id] = mmem

            # updating mmem
            self.sim.simSetJointMode(handle, 5, 0)

            load     = self.sim.simJointGetForce(handle)[0]
            position = self.sim.simGetJointPosition(handle)[0]
            #velocity = self.sim.simGetJointVelocity(handle)[0]

            # update position
            pos_deg = self.rad2deg(position)
            raw_position = conversions.deg_2raw(pos_deg, mmem)
            mmem[protocol.REG_ADDRESS("PRESENT_POSITION")] = raw_position
            mmem[protocol.REG_ADDRESS("GOAL_POSITION")]    = raw_position

            # update load
            load_percent = min(100.0, MAXTORQUE[mmem.model])
            raw_load = conversions.present_load_2raw(load_percent, mmem)
            mmem[protocol.REG_ADDRESS("PRESENT_LOAD")] = raw_load

            # velocity is 0

            mmems.append(mmem)

        #self.sim.speSetMultipleJointTargetPosition(handles, positions)

        return mmems

    # We assume RX limits
    @staticmethod
    def deg2rad(val):
        return math.pi*(val-150.0)/180.0

    @staticmethod
    def rad2deg(val):
        return 180.0*val/math.pi+150.0

    @staticmethod
    def rawdps2rads(val):
        return math.pi*val/180.0*6*0.111

    @staticmethod
    def rads2rawspeed(val):
        return round(val*180.0/math.pi/6/0.111 + 1024)

    def _set_background_fun(self, motor_ids):
        """
            Register all continuously called functions.
        """
        self.handles = [self.id2handle[motor_id] for motor_id in motor_ids]
        self.sim.registerBackgroundFunction("speSetMultipleJointTargetPosition", self.handles)
        self.sim.registerBackgroundFunction("speGetMultipleJointPosition", self.handles)

        # self.sim.registerBackgroundFunction("simSetJointForce", handle)

        # Useless for now
        # self.sim.registerBackgroundFunction("simJointGetForce", handle)
        self.sim.registerBackgroundFunction("simSetDynamicMotorUpperLimitVelocity", self.handles)
        self.sim.registerBackgroundFunction("speSetMultipleDynamicMotorUpperLimitVelocity", self.handles)


    # MARK Parameter based read/write

    supported_set = {'GOAL_POSITION' : '_set_position',
                     'TORQUE_LIMIT'  : '_set_torque_limit',
                     'MOVING_SPEED'  : '_set_moving_speed'
                    }

    supported_get = {'PRESENT_POSITION'       : '_get_position',
                     'PRESENT_SPEED'          : '_get_speed',
                     'PRESENT_LOAD'           : '_get_load',
                     'MOVING'                 : '_get_speed',
                     'PRESENT_POS_SPEED_LOAD' : '_get_pos_speed_load',
    }


    def set(self, motor_id, control_name, value):
        """Send a write instruction to a motor regarding control_name"""

        try:
            setter = getattr(self, self.supported_set[control_name])
        except KeyError:
            setter = self._set_dummy
        setter(motor_id, value)

    def get(self, motor_id, control_name):
        """Send a read instruction to a motor regarding control_name"""

        try:
            getter = getattr(self, self.supported_get[control_name])
        except KeyError:
            getter = self._get_dummy
        getter(motor_id)


    def _set_dummy(self, motor_id, value):
        pass

    def _get_dummy(self, motor_id):
        pass

    def _set_position(self, motor_id, value):
        raise DeprecationWarning
        handle = self.id2handle[motor_id]
        mmem = self.motormems[motor_id]
        pos_deg = conversions.raw2_deg(value, mmem)
        pos_rad = self.deg2rad(pos_deg)
        if self.sim.simSetJointTargetPosition(handle, pos_rad) == -1:
            print('warning: problem setting position')

        mmem[protocol.REG_ADDRESS('GOAL_POSITION')] = value

    def _get_position(self, motor_id):
        raise DeprecationWarning
        handle = self.id2handle[motor_id]
        mmem = self.motormems[motor_id]
        pos_rad = self.sim.simGetJointPosition(handle)[0]
        pos_deg = self.rad2deg(pos_rad)

        mmem[protocol.REG_ADDRESS('PRESENT_POSITION')] = conversions.deg_2raw(pos_deg, mmem)

        return

    def _get_sync_position(self, motor_ids):

        pos_rads = self.sim.speGetMultipleJointPosition(len(self.handles), self.handles)
        for pos_rad, handle in zip(pos_rads, self.handles):
            motor_id = self.handle2id[handle]
            mmem = self.motormems[motor_id]
            pos_deg = self.rad2deg(pos_rad)
            mmem[protocol.REG_ADDRESS('PRESENT_POSITION')] = conversions.deg_2raw(pos_deg, mmem)

    def _set_sync_position(self, id_pos_pairs):
        pos_rads = []

        id_pos_dict = dict(id_pos_pairs)
        for handle in self.handles:
            motor_id = self.handle2id[handle]
            mmem = self.motormems[motor_id]

            value = id_pos_dict.get(motor_id, mmem[protocol.REG_ADDRESS('GOAL_POSITION')])
            pos_deg = conversions.raw2_deg(value, mmem)
            pos_rads.append(self.deg2rad(pos_deg))

        if self.sim.speSetMultipleJointTargetPosition(len(self.handles), self.handles, pos_rads) != 0:
            # do it anyway, else, the set is lost.
            for motor_id, value in id_pos_pairs:
                mmem = self.motormems[motor_id]
                mmem[protocol.REG_ADDRESS('GOAL_POSITION')] = value
        else:
            for motor_id, value in id_pos_pairs:
                mmem = self.motormems[motor_id]
                mmem[protocol.REG_ADDRESS('GOAL_POSITION')] = value

    def _set_sync_speed(self, id_pos_pairs):
        return
        speeds_rads = []

        id_speed_dict = dict(id_speed_pairs)
        for handle in self.handles:
            motor_id = self.handle2id[handle]
            mmem = self.motormems[motor_id]

            value = id_speed_dict.get(motor_id, mmem[protocol.REG_ADDRESS('MOVING_SPEED')])
            rads_value = self.rawdps2rads(value)

            speed_rads.append(rads_value)

        if self.sim.speSetMultipleDynamicMotorUpperLimitVelocity(len(self.handles), self.handles, speed_rads) != 0:
            # do it anyway, else, the set is lost.
            for motor_id, value in id_speed_pairs:
                mmem = self.motormems[motor_id]
                mmem[protocol.REG_ADDRESS('MOVING_SPEED')] = value
        else:
            for motor_id, value in id_speed_pairs:
                mmem = self.motormems[motor_id]
                mmem[protocol.REG_ADDRESS('MOVING_SPEED')] = value

    def _set_speed(self, motor_id, speed):
        handle = self.id2handle[motor_id]
        mmem = self.motormems[motor_id]
        rads_speed = self.rawdps2rads(speed)
        if self.sim.simSetDynamicMotorUpperLimitVelocity(handle, rads_speed) == -1:
            print('warning: problem setting position')

        mmem[protocol.REG_ADDRESS('MOVING_SPEED')] = speed


    def _get_speed(self, motor_id):
        return
        handle = self.id2handle[motor_id]
        mmem = self.motormems[motor_id]
        vel_rad = self.sim.simGetJointVelocity(handle)[0]

        mmem[protocol.REG_ADDRESS('MOVING_SPEED')] = self.rads2rawspeed(vel_deg, mmem)

    def _get_load(self, motor_id):
        return
        handle = self.id2handle[motor_id]
        mmem = self.motormems[motor_id]

        load = self.sim.simJointGetForce(handle)[0]
        load_percent = min(100.0, load/MAXTORQUE[mmem.model])
        raw_load = conversions.present_load_2raw(load_percent, mmem)
        mmem[protocol.REG_ADDRESS("PRESENT_LOAD")] = raw_load

    def _set_torque_limit(self, motor_id, value):
        return
        handle = self.id2handle[motor_id]
        mmem = self.motormems[motor_id]

        torque_percent = conversions.raw2_torque(value, mmem)
        self.sim.simSetJointForce(handle, torque_percent*MAXTORQUE[mmem.model])

    def _get_pos_speed_load(self, motor_id):
        return
        self._get_position(motor_id)
        self._get_speed(motor_id)
        self._get_load(motor_id)


    # MARK Mode

    def change_mode(self, motor_id, mode):
        """.. warning: No effect in V-REP simulations."""
        pass

    def set_to_wheel_mode(self, motor_id):
        """.. warning: No effect in V-REP simulations."""
        pass

    def set_to_joint_mode(self, motor_id, angle_limits = None):
        """.. warning: No effect in V-REP simulations."""
        pass


    # MARK: - Sync Read/Write

    def get_sync_positions(self, motor_ids):
        """
            Synchronizes the getting of positions in degrees of all motors specified.

            :param motor_ids: specified motor ids [0-253]
            :type motor_ids: list of ids
            """
        self._get_sync_position(motor_ids)

    def set_sync_positions(self, id_pos_pairs):
        """
            Synchronizes the setting of the specified positions (in degrees) to the motors.

            :type id_pos_pairs: list of couple (motor id, position)

            """
        self._set_sync_position(id_pos_pairs)

    set_sync_goal_position = set_sync_positions

    def get_sync_speeds(self, motor_ids):
        """
            Synchronizes the getting of speed in dps of all motors specified.

            :param motor_ids: specified motor ids [0-253]
            :type motor_ids: list of ids

            .. warning:: This method only works with the USB2AX.

            """
        for motor_id in motor_ids:
            self._get_speed(motor_id)

    def set_sync_speeds(self, id_speed_pairs):
        """
            Synchronizes the setting of the specified speeds (in dps) to the motors.

            :type id_speed_pairs: list of couple (motor id, speed)

            """
        for motor_id, value in id_speed_pairs:
            self._set_position(motor_id, value)


    def get_sync_loads(self, motor_ids):
        """
            Synchronizes the getting of load in percentage of all motors specified.

            :param motor_ids: specified motor ids [0-253]
            :type motor_ids: list of ids

            .. warning:: This method only works with the USB2AX.

            """
        for motor_id in motor_ids:
            self._get_load(motor_id)

    def set_sync_torque_limits(self, id_torque_pairs):
        """
            Synchronizes the setting of the specified torque limits to the motors.

            :type id_torque_pairs: list of couple (motor id, torque)

            """
        for motor_id, value in id_torque_pairs:
            self._set_torque_limit(motor_id, value)


    def get_sync_positions_speeds_loads(self, motor_ids):
        """
            Synchronizes the getting of positions, speeds, load of all motors specified.

            :param motor_ids: specified motor ids [0-253]
            :type motor_ids: list of ids
            :return: list of (position, speed, load)

            """

        self._get_sync_position(motor_ids)

        for motor_id in motor_ids:
            self._get_speed(motor_id)
            self._get_load(motor_id)

    def set_sync_speeds_torque_limits(self, id_speed_torque_tuples):
        """
            Synchronizes the setting of the specified speeds and torque limits (in their respective units) to the motors.

            * The speed is expressed in dps (positive values correspond to clockwise).
            * The torque limit is expressed as a percentage of the maximum torque.

            :param id_speed_torque_tuples: each value must be expressed in its own units.
            :type id_speed_torque_tuples: list of (motor id, speed, torque)

            """

        for motor_id, speed, torque in id_speed_torque_tuples:
            self._set_speed(motor_id, speed)
            self._set_torque_limit(motor_id, torque)

    def set_sync_positions_speeds_torque_limits(self, id_pos_speed_torque_tuples):
        """
            Synchronizes the setting of the specified positions, speeds and torque limits (in their respective units) to the motors.

            * The position is expressed in degrees.
            * The speed is expressed in dps (positive values correspond to clockwise).
            * The torque limit is expressed as a percentage of the maximum torque.

            :param id_pos_speed_torque_tuples: each value must be expressed in its own units.
            :type id_pos_speed_torque_tuples: list of (motor id, position, speed, torque)

            """

        self._set_sync_position(tuple((motor_id, pos) for motor_id, pos, speed, torque in id_pos_speed_torque_tuples))
        self._set_sync_speed(tuple((motor_id, speed) for motor_id, pos, speed, torque in id_pos_speed_torque_tuples))

        for motor_id, pos, speed, torque in id_pos_speed_torque_tuples:
            self._set_speed(motor_id, speed)
            self._set_torque_limit(motor_id, torque)


    # MARK - Special cases

    def change_id(self, motor_id, new_motor_id):
        """.. warning: No effect in V-REP simulations."""
        pass

    def get_status_return_level(self, motor_id):
        """.. warning: No effect in V-REP simulations."""
        pass

    def lock_eeprom(self, motor_id):
        """.. warning: No effect in V-REP simulations."""
        pass

    def unlock_eeprom(self, motor_id):
        """.. warning: No effect in V-REP simulations."""
        pass

# MARK: - Dxl Error

class DynamixelCommunicationError(Exception):
    def __init__(self, message, instruction_packet, response):
        self.message = message
        self.instruction_packet = instruction_packet
        self.response = map(ord, response) if response else None

    def __str__(self):
        return '%s (instruction packet: %s, status packet: %s)' \
            % (self.message, self.instruction_packet, self.response)
