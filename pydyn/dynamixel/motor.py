"""
The motor classes allow you to control and read the motor values in a
asynchronous manner.

When reading a value, the cached (most up to date) value is returned. For
EEPROM and unchanging RAM values (eg compliance slopes), this is not a problem.
For speed, pos and load, that means that the values of the last loop of the
controller are provided. Depending on the current speed of the motor and the
speed of the serial loop, these values might not reflect accurrately the actual
status of the motor, but this is the best the hardware is capable to do. Some
other RAM values may change quickly and often (``torque_enable``, ``led``,
``present_voltage``, ``present_temperature``, ``registered``, ``moving`` and
``current`` (or ``sensed_current``)), and if an updated value is desired, the
user should use the request_read() method, with the previous names as argument.

Writing a value is not immediate, and will be done during the next loop.

Given a motor instance ``m`` and a variable (eg, ``torque_enable``), you can
read the cached value, request refreshing the cached value and writting a new
value using, respectively :

>>> m.torque_enable
>>> m.request_read('torque_enable')
>>> m.torque_enable = False

Once a request has been made, you can call ``m.requested('torque_enable')`` to
check what value is going to be written.

The requested value can be read with the method m.requested('torque_enable'),
and may differ from m.torque_enable until the motor value has been updated on
the hardware. If the request have already been processed, requested() will
return None. Note that if you request multiple values for the same variable,
only the last one at the start of the next controller loop will be taken into
account.

For every variable, there is an alternative bytes variable if the unit or value
differs (no bytes for id, firmware for example). You can use them to access the
bytes integer value present in the hardware memory register.
"""


import threading
import collections
import textwrap

from .. import color
from ..refs import protocol as pt
from ..refs import limits
from ..refs import conversions as conv
from ..refs import exc

class ROByteMotorControl(object):
    def __init__(self, control, doc=''):
        self.control = control
        self.__doc__ = textwrap.dedent(doc)

    def __get__(self, instance, owner):
        if instance is None:
            return self
        return instance.mmem[self.control]

    def __set__(self, instance, value):
        raise AttributeError("can't set {}".format(self.control.name))

class RWByteMotorControl(ROByteMotorControl):

    def __set__(self, instance, value):
        limits.CHECK_BYTES[self.control](value, modelclass=instance.modelclass,
                                                      mode=instance.mode)
        instance._register_write(self.control, value)

class ROMotorControl(ROByteMotorControl):

    def __get__(self, instance, owner):
        if instance is None:
            return self
        byte_value = instance.mmem[self.control]
        return conv.CONV[self.control][1](byte_value, modelclass=instance.modelclass,
                                                      mode=instance.mode)

class RWMotorControl(ROMotorControl):

    def __set__(self, instance, value):
        limits.CHECK[self.control](value, modelclass=instance.modelclass,
                                                mode=instance.mode)
        byte_value = conv.CONV[self.control][0](value, modelclass=instance.modelclass,
                                                       mode=instance.mode)
        instance._register_write(self.control, byte_value)


class Motor(object):

    MotorError = exc.MotorError

    def __init__(self, memory):

        object.__setattr__(self, 'mmem', memory) # self.mmem = memory

        # these dictionaries collect write or read request on part of non standart
        # memory (not pos, speed, load (read) or torque lim, moving speed goal pos (write))
        object.__setattr__(self, 'read_requests', collections.OrderedDict()) # self.read_requests  = collections.OrderedDict()
        object.__setattr__(self, 'write_requests', collections.OrderedDict()) # self.write_requests  = collections.OrderedDict()
        # the dictionary is protected by a semaphore since both Motor and
        # Controller access and modify it. But the semaphore should only be acquired for
        # the duration of atomic actions on the dictionary, and not, for instance, the
        # serial communication related to a request.
        object.__setattr__(self, 'request_lock', threading.Lock()) # self.request_lock = threading.Lock()

        # backing up angle limits to switch between joint and wheel mode
        object.__setattr__(self, '_joint_angle_limits_bytes', self.angle_limits_bytes) # self._joint_angle_limits_bytes = self.angle_limits_bytes

        # if a motor error is detected on the motor, a MotorError instance will be put in this list.
        object.__setattr__(self, '_error', [])

    def __repr__(self):
        return 'M{}'.format(self.id)

    def __cmp__(self, other):
        if self.id < other.id:
            return -1
        elif self.id == other.id:
            return  0
        else:
            return  1

    def __setattr__(self, attr, value):
        if hasattr(self, attr):
            object.__setattr__(self, attr, value)
        else:
            raise AttributeError(("'{}' assignement error. This attribute does not exist "
                                  "and new attributes cannot be defined on Motor "
                                  "instances.").format(attr, attr))


    # MARK Read/Write request

    aliases_read = {
        'voltage'         : 'present_voltage',
        'temperature'     : 'present_temperature',
        'position'        : 'present_position',
        'speed'           : 'present_speed',
        'load'            : 'present_load',
    }

    aliases_write = {
        'position'        : 'goal_position',
        'speed'           : 'moving_speed',
    }

    def __copy__(self):
        raise RuntimeError("Motor instances can't be copied")

    def __deepcopy__(self, memo=None):
        raise RuntimeError("Motor instances can't be copied")

    def _str2ctrl(self, control, aliases=()):
        """\
        Return the Control instance corresponding to the string.
        If already a control, return unchanged.
        :param aliases:  aliases dict for translations
        """
        if not isinstance(control, pt.Control):
            control = control.lower()
            if control in self.aliases:
                control = self.aliases[control]
            control = pt.CTRL[control.upper()]
        return control

    def request_read(self, control):
        """\
        Request a specific value of the motor to be refreshed

        :arg control:  the name of the value. See the :py:mod:`protocol <pydyn.refs.protocol>` module for the list of values.
        """
        control = self._str2ctrl(control, aliases=Motor.aliases_read)

        self.request_lock.acquire()
        try:
            if control in self.read_requests:
                self.read_requests.pop(control) # so that order is kept consistent
            self.read_requests[control] = True
        except Exception as e:
            self.request_lock.release()
            raise e
        self.request_lock.release()

    def request_write(self, control, value):
        """\
        Request a specific write on the motor

        :arg name:  the name of the value. See the :py:mod:`protocol <pydyn.refs.protocol>` module for the list of values.
        """
        for err in self._error:
            raise err
        control = self._str2ctrl(control, aliases=Motor.aliases_write)
        setattr(self, control, value) # for bound checking, mode handling

    def _register_write(self, control, val): # TODO this is a bad name for ACTION/REGISTERED
        """Register the write request for the controller benefit"""
        for err in self._error:
            raise err
        self.request_lock.acquire()
        try:
            if control in self.write_requests:
                self.write_requests.pop(control) # so that order is kept consistent
            self.write_requests[control] = val
        except BaseException as e:
            self.request_lock.release()
            raise e
        self.request_lock.release()


    def requested_read(self, control):
        """\
        Return True if a value is requested for reading but was not read yet.
        After the value was read, return False.
        """
        control = self._str2ctrl(control, aliases=Motor.aliases_read)

        self.request_lock.acquire()
        try:
            value = self.read_requests.get(control, False)
        except BaseException as e:
            self.request_lock.release()
            raise e
        self.request_lock.release()
        return value

    def requested_write(self, control):
        """\
        Return the value of the requested write, if it wasn't written yet.
        After the value was read, return None.
        """
        control = self._str2ctrl(control, aliases=Motor.aliases_write)

        self.request_lock.acquire()
        value = self.write_requests.get(control, False)
        self.request_lock.release()
        return value

    # MARK Mode

    @property
    def mode(self):
        """"""
        return self.mmem.mode

    @mode.setter
    def mode(self, value):
        """\
        This will change the mode of the motor.
        Previous joint angle limits will be restored when coming from wheel mode.

        :param value:  either 'joint' or 'wheel'
        """
        limits.checkoneof('mode', ['wheel', 'joint'], value)
        if value == 'wheel':
            self._joint_angle_limits_bytes = self.angle_limits_bytes
            self._register_write(pt.ANGLE_LIMITS, (0, 0))
        else:
            self._register_write(pt.ANGLE_LIMITS, self._joint_angle_limits_bytes)


    # MARK EEPROM properties

    _doc_id = """\
    The id of the motor, valid value are between 0 and 253.

    :raise ValueError:  if you set an id that is already used.
    """
    id_bytes = RWByteMotorControl(pt.ID, doc=_doc_id)
    id       =     RWMotorControl(pt.ID, doc=_doc_id)

    _doc_model_bytes = """The model of the motor."""
    model_bytes = ROByteMotorControl(pt.MODEL_NUMBER, doc=_doc_model_bytes)
    _doc_model       = """The model of the motor as a string (eg. 'AX-12')."""
    model       =     ROMotorControl(pt.MODEL_NUMBER, doc=_doc_model)

    _doc_firmware_bytes = """The version of the firmware."""
    firmware_bytes = ROByteMotorControl(pt.FIRMWARE)
    firmware       =     ROMotorControl(pt.FIRMWARE)

    @property
    def modelclass(self):
        """ The class of motor (for a RX-28, return 'RX') """
        return self.mmem.modelclass

    # MARK Baudrate
    _doc_baudrate_bytes = """\
    Baudrate for the motor

    Usual values are 1, 3, 4, 7, 9, 16, 34, 103, 207 (and 0, 250, 251, 252 for
    MX), but anything between 1 and 253 is accepted.

    .. warning:: if you change the baudrate, the motor will be unreachable
                 until you change the connection baudrate too.
    """
    _doc_baudrate = """\
    Baudrate for the motor

    Usual values are 1000000, 50000, 40000, 25000, 20000, 115200, 57600,
    19200, 9600 (and 2000000, 2250000, 2500000, 3000000 for MX), but anything
    between 1000000 and 7874 is accepted.

    .. warning:: if you change the baudrate, the motor will be unreachable
                 until you change the connection baudrate too.
    """
    baudrate_bytes = RWByteMotorControl(pt.BAUDRATE, doc=_doc_baudrate_bytes)
    baudrate       = RWMotorControl(pt.BAUDRATE, doc=_doc_baudrate)

    # MARK Return Delay Time
    return_delay_time_bytes = RWByteMotorControl(pt.RETURN_DELAY_TIME)
    return_delay_time       =     RWMotorControl(pt.RETURN_DELAY_TIME)

    # MARK Angle Limits

    @property
    def cw_angle_limit(self):
        return conv.CONV[pt.CW_ANGLE_LIMIT][1](self.cw_angle_limit_bytes, modelclass=self.modelclass, mode=self.mode)

    @property
    def cw_angle_limit_bytes(self):
        return self.mmem[pt.CW_ANGLE_LIMIT]

    @cw_angle_limit.setter
    def cw_angle_limit(self, val):
        self.cw_angle_limit_bytes = conv.CONV[pt.CCW_ANGLE_LIMIT][0](val, modelclass=self.modelclass, mode=self.mode)

    @cw_angle_limit_bytes.setter
    def cw_angle_limit_bytes(self, val):
        limits.CHECK_BYTES[pt.CW_ANGLE_LIMIT](val, modelclass=self.modelclass, mode=self.mode)
        if int(val) > int(self.ccw_angle_limit_bytes):
            raise ValueError('CW angle limit ({}) should be inferior to CCW angle limit ({})'.format(val, self.ccw_angle_limit_bytes))
        if val != 0 or val != self.ccw_angle_limit_bytes:
            self._joint_angle_limits_bytes = (val, self.ccw_angle_limit_bytes)
        self._register_write(pt.CW_ANGLE_LIMIT, val)

    @property
    def ccw_angle_limit(self):
        return conv.CONV[pt.CCW_ANGLE_LIMIT][1](self.ccw_angle_limit_bytes, modelclass=self.modelclass, mode=self.mode)

    @property
    def ccw_angle_limit_bytes(self):
        return self.mmem[pt.CCW_ANGLE_LIMIT]

    @ccw_angle_limit.setter
    def ccw_angle_limit(self, val):
        self.ccw_angle_limit_bytes = conv.ccw_angle_limit_2bytes(val, modelclass=self.modelclass, mode=self.mode)

    @ccw_angle_limit_bytes.setter
    def ccw_angle_limit_bytes(self, val):
        limits.CHECK_BYTES[pt.CCW_ANGLE_LIMIT](val, modelclass=self.modelclass, mode=self.mode)
        #if int(val[0]) > int(self.ccw_angle_limit_bytes):
        #    raise ValueError('CW angle limit ({}) should be inferior to CCW angle limit ({})'.format(val[0], self.ccw_angle_limit_bytes))
        if val != 0 or val != self.cw_angle_limit_bytes:
            self._joint_angle_limits_bytes = (val, self.cw_angle_limit_bytes)
        self._register_write(pt.CCW_ANGLE_LIMIT, val)


    @property
    def angle_limits(self):
        return self.cw_angle_limit, self.ccw_angle_limit

    @property
    def angle_limits_bytes(self):
        return self.cw_angle_limit_bytes, self.ccw_angle_limit_bytes

    @angle_limits.setter
    def angle_limits(self, val):
        self.angle_limits_bytes = conv.cw_angle_limit_2bytes(val[0], modelclass=self.modelclass, mode=self.mode), conv.ccw_angle_limit_2bytes(val[1], modelclass=self.modelclass, mode=self.mode)

    @angle_limits_bytes.setter
    def angle_limits_bytes(self, val):
        limits.checkbounds('cw_angle_limit', 0, limits.POSITION_RANGES[self.modelclass][0], int(val[0]))
        limits.checkbounds('ccw_angle_limit', 0, limits.POSITION_RANGES[self.modelclass][0], int(val[1]))
        assert len(val) == 2
        #if val[0] > val[1]:
        #    raise ValueError('CW angle limit ({}) should be inferior to CCW angle limit ({})'.format(val[0], val[1]))
        if val[0] != 0 or val[1] != 0:
            self._joint_angle_limits_bytes = val
        self._register_write(pt.ANGLE_LIMITS, val)


    # MARK Highest Limit Temperature
    highest_limit_temperature_bytes = RWByteMotorControl(pt.HIGHEST_LIMIT_TEMPERATURE)
    highest_limit_temperature       =     RWMotorControl(pt.HIGHEST_LIMIT_TEMPERATURE)

    # MARK Limit Voltage
    highest_limit_voltage_bytes = RWByteMotorControl(pt.HIGHEST_LIMIT_VOLTAGE)
    highest_limit_voltage       =     RWMotorControl(pt.HIGHEST_LIMIT_VOLTAGE)

    lowest_limit_voltage_bytes = RWByteMotorControl(pt.LOWEST_LIMIT_VOLTAGE)
    lowest_limit_voltage       =     RWMotorControl(pt.LOWEST_LIMIT_VOLTAGE)

    # MARK Max Torque
    max_torque_bytes = RWByteMotorControl(pt.MAX_TORQUE)
    max_torque       =     RWMotorControl(pt.MAX_TORQUE)

    # MARK Return Status Level
    status_return_level_bytes = RWByteMotorControl(pt.STATUS_RETURN_LEVEL)
    status_return_level       =     RWMotorControl(pt.STATUS_RETURN_LEVEL)

    # MARK Alarms
    alarm_led_bytes = RWByteMotorControl(pt.ALARM_LED)
    alarm_shutdown_bytes = RWByteMotorControl(pt.ALARM_SHUTDOWN)


    # MARK RAM properties

    # MARK Torque Enable

    torque_enable_bytes = RWByteMotorControl(pt.TORQUE_ENABLE)
    torque_enable       =     RWMotorControl(pt.TORQUE_ENABLE)

    @property
    def compliant(self):
        """compliant is a more intuive alternative for torque_enable
        compliant == not torque_enable.
        """
        return not self.torque_enable

    @compliant.setter
    def compliant(self, val):
        self.torque_enable = not val

    @property
    def compliant_bytes(self):
        return 0 if self.torque_enable else 1

    @compliant.setter
    def compliant_bytes(self, val):
        self.torque_enable_bytes = {0:1, 1:0}[val]


    # MARK LED
    led_bytes = RWByteMotorControl(pt.LED)
    led       =     RWMotorControl(pt.LED)

    # MARK Goal position
    goal_position_bytes = RWByteMotorControl(pt.GOAL_POSITION)
    goal_position       =     RWMotorControl(pt.GOAL_POSITION)

    # MARK moving_speed
    moving_speed_bytes = RWByteMotorControl(pt.MOVING_SPEED)
    moving_speed       =     RWMotorControl(pt.MOVING_SPEED)

    # MARK torque_limit
    torque_limit_bytes = RWByteMotorControl(pt.TORQUE_LIMIT)
    torque_limit       =     RWMotorControl(pt.TORQUE_LIMIT)


    # MARK present_position, present_speed, present_load
    present_position_bytes = ROByteMotorControl(pt.PRESENT_POSITION)
    present_position       =     ROMotorControl(pt.PRESENT_POSITION)

    present_speed_bytes = ROByteMotorControl(pt.PRESENT_SPEED)
    present_speed       =     ROMotorControl(pt.PRESENT_SPEED)

    present_load_bytes = ROByteMotorControl(pt.PRESENT_LOAD)
    present_load       =     ROMotorControl(pt.PRESENT_LOAD)

    # MARK position, speed

    # Here you can read and set position and speed through only two attribute.
    # You're missing out on timing subtelty, and it may lead to unexpected
    # behavior. But in most case, you're probably fine.

    @property
    def position(self):
        return self.present_position

    @property
    def position_bytes(self):
        return self.present_position_bytes

    @position.setter
    def position(self, val):
        self.goal_position = val

    @position_bytes.setter
    def position_bytes(self, val):
        self.goal_position_bytes = val

    @property
    def speed(self):
        return self.present_speed

    @property
    def speed_bytes(self):
        return self.present_speed_bytes

    @speed.setter
    def speed(self, val):
        self.moving_speed = val

    @speed_bytes.setter
    def speed_bytes(self, val):
        self.moving_speed_bytes = val

    # position_bytes = property(present_position_bytes.__get__, goal_position_bytes.__set__)
    # position       = property(present_position.__get__,       goal_position.__set__)

    # speed_bytes = property(present_speed_bytes.__get__, moving_speed_bytes.__set__)
    # speed       = property(present_speed.__get__,       moving_speed.__set__)

    load = present_load
    load_bytes = present_load_bytes


    # MARK Present voltage
    present_voltage_bytes = ROByteMotorControl(pt.PRESENT_VOLTAGE)
    present_voltage       =     ROMotorControl(pt.PRESENT_VOLTAGE)

    voltage_bytes = present_voltage_bytes
    voltage       = present_voltage

    # MARK Present temperature
    present_temperature_bytes = ROByteMotorControl(pt.PRESENT_TEMPERATURE)
    present_temperature       =     ROMotorControl(pt.PRESENT_TEMPERATURE)

    temperature_bytes = present_temperature_bytes
    temperature       = present_temperature

    # MARK Registered
    registered_bytes = ROByteMotorControl(pt.REGISTERED)
    registered       =     ROMotorControl(pt.REGISTERED)


    # MARK Lock EEPROM

    @property
    def lock(self):
        return conv.bytes2_lock(self.registered_bytes)

    @property
    def lock_bytes(self):
        return self.mmem[pt.LOCK]

    # TODO Locking
    @lock.setter
    def lock(self, val):
        limits.checkoneof('lock', [0, 1, True, False], val)

    @lock_bytes.setter
    def lock_bytes(self, val):
        """Locking EEPROM makes changing EEPROM values impossible.

        Once the EEPROM is locked, only a power cycle can unlock it.

        .. raise ValueError: if attempting to unlock a locked EEPROM.
        """
        limits.checkoneof('lock', [0, 1], val)
        if val == 0:
            if self.lock_bytes == 1:
                raise ValueError('to unlock the eeprom, you should cycle power')
        else:
            self._register_write(pt.TORQUE_LIMIT, 1)



    # MARK Moving
    moving_bytes = ROByteMotorControl(pt.MOVING)
    moving       =     ROMotorControl(pt.MOVING)

    # MARK Punch
    punch_bytes = RWByteMotorControl(pt.PUNCH)
    punch       =     RWMotorControl(pt.PUNCH)


    # MARK Printing EEPROM, RAM
    # TODO move to Motor

    def _mem_desc(self, addr, desc):
        return ('{}{:2d}{}'.format(color.cyan, addr, color.end),
                '{}{:4d}  {}{}{}'.format(color.purple, self.mmem[addr], color.grey, desc, color.end))

    def eeprom_desc(self):
        s = ['EEPROM',
             '{} Model                 : {}'.format(*self._mem_desc(0,  self.model)),
             '{} Firware               : {}'.format(*self._mem_desc(2,  '')),
             '{} ID                    : {}'.format(*self._mem_desc(3,  '')),
             '{} Baud Rate             : {}'.format(*self._mem_desc(4,  '{} bauds'.format(self.baudrate))),
             '{} Return Delay Time     : {}'.format(*self._mem_desc(5,  '{} usec'.format(self.return_delay_time))),
             '{} CW Angle Limit        : {}'.format(*self._mem_desc(6,  '{:6.2f} degrees'.format(self.cw_angle_limit))),
             '{} CCW Angle Limit       : {}'.format(*self._mem_desc(8,  '{:6.2f} degrees, {} mode'.format(self.ccw_angle_limit, self.mode))),
             '{} Max Limit Temp        : {}'.format(*self._mem_desc(11, '{} celsius'.format(self.highest_limit_temperature))),
             '{} Min Limit Voltage     : {}'.format(*self._mem_desc(12, '{:4.1f} V'.format(self.lowest_limit_voltage))),
             '{} Max Limit Voltage     : {}'.format(*self._mem_desc(13, '{:4.1f} V'.format(self.highest_limit_voltage))),
             '{} Max Torque            : {}'.format(*self._mem_desc(14, '{:.1f} % of max'.format(self.max_torque))),
             '{} Status Return Level   : {}'.format(*self._mem_desc(16, '')),
             '{} Alarm LED             : {}'.format(*self._mem_desc(17, '')),
             '{} Alarm Shutdown        : {}'.format(*self._mem_desc(18, '')),
             '\n'+color.end]
        s = '\n'.join(s)
        return s

    def ram_desc(self):
        s  = ['RAM',
              '{} Torque Enable         : {}'.format(*self._mem_desc(24, self.torque_enable)),
              '{} LED                   : {}'.format(*self._mem_desc(25, self.led)),
             ]
        #s += self._pid_ram_desc() # PID or Margin/Slopes
        s += [
              '{} Goal Position         : {}'.format(*self._mem_desc(30,  '{:6.2f} degrees'.format(self.goal_position))),
              '{} Moving Speed          : {}'.format(*self._mem_desc(32,  '{:6.1f} dps'.format(self.moving_speed))),
              '{} Torque Limit          : {}'.format(*self._mem_desc(34,  '{:6.1f} % of max'.format(self.torque_limit))),
              '{} Present Position      : {}'.format(*self._mem_desc(36,  '{:6.2f} degree'.format(self.present_position))),
              '{} Present Speed         : {}'.format(*self._mem_desc(38,  '{:6.1f} dps'.format(self.present_speed))),
              '{} Present Load          : {}'.format(*self._mem_desc(40,  '{:6.1f} % of max {}'.format(abs(self.present_load), ['ccw', 'cw'][self.present_load > 0]))),
              '{} Present Voltage       : {}'.format(*self._mem_desc(42,  '{} V'.format(self.present_voltage))),
              '{} Present Temperature   : {}'.format(*self._mem_desc(43,  '{} celsius'.format(self.present_temperature))),
              '{} Registered            : {}'.format(*self._mem_desc(44,  self.registered)),
              '{} Moving                : {}'.format(*self._mem_desc(46,  self.moving)),
              '{} Lock                  : {}'.format(*self._mem_desc(46,  self.lock)),
              '{} Punch                 : {}'.format(*self._mem_desc(46,  self.punch)),
             '\n'+color.end]
        s = '\n'.join(s)
        return s


    # MARK Extra motor properties

# Here we don't inherit from Motor - we do multiple inheritance (composition)
# instead that grant us more flexibility
class ComplianceMarginSlopeExtra(object):

    # MARK Compliance margin
    cw_compliance_margin_bytes  = RWByteMotorControl(pt.CW_COMPLIANCE_MARGIN)
    cw_compliance_margin        =     RWMotorControl(pt.CW_COMPLIANCE_MARGIN)

    ccw_compliance_margin_bytes = RWByteMotorControl(pt.CCW_COMPLIANCE_MARGIN)
    ccw_compliance_margin       =     RWMotorControl(pt.CCW_COMPLIANCE_MARGIN)

    compliance_margins_bytes    = RWByteMotorControl(pt.COMPLIANCE_MARGINS)
    compliance_margins          =     RWMotorControl(pt.COMPLIANCE_MARGINS)

    # MARK compliance slopes
    cw_compliance_slope_bytes   = RWByteMotorControl(pt.CW_COMPLIANCE_SLOPE)
    cw_compliance_slope         =     RWMotorControl(pt.CW_COMPLIANCE_SLOPE)

    ccw_compliance_slope_bytes  = RWByteMotorControl(pt.CCW_COMPLIANCE_SLOPE)
    ccw_compliance_slope        =     RWMotorControl(pt.CCW_COMPLIANCE_SLOPE)

    compliance_slopes_bytes     = RWByteMotorControl(pt.COMPLIANCE_SLOPES)
    compliance_slopes           =     RWMotorControl(pt.COMPLIANCE_SLOPES)


class PIDExtra(object):

    # MARK PID Gains
    d_gain_bytes     = RWByteMotorControl(pt.D_GAIN)
    d_gain           =     RWMotorControl(pt.D_GAIN)

    p_gain_bytes     = RWByteMotorControl(pt.P_GAIN)
    p_gain           =     RWMotorControl(pt.P_GAIN)

    i_gain_bytes     = RWByteMotorControl(pt.I_GAIN)
    i_gain           =     RWMotorControl(pt.I_GAIN)

    gains_bytes      = RWByteMotorControl(pt.GAINS)
    gains            =     RWMotorControl(pt.GAINS)


class SensedCurrentExtra(object):

    # MARK Sensed Current
    sensed_current_bytes = ROByteMotorControl(pt.SENSED_CURRENT)
    sensed_current       =     ROMotorControl(pt.SENSED_CURRENT)


class CurrentExtra(object):

    # MARK Current
    current_bytes = RWByteMotorControl(pt.CURRENT)
    current       =     RWMotorControl(pt.CURRENT)
    # Only the MX64 and 106 seems to support current, although you have to go
    # through the korean doc for the MX64 to know that. We assume it is a
    # mistake and all MX support it.


class TorqueModeExtra(object):

    # MARK Torque Control Mode
    torque_control_mode_enable_bytes = RWByteMotorControl(pt.TORQUE_CONTROL_MODE_ENABLE)
    torque_control_mode_enable       =     RWMotorControl(pt.TORQUE_CONTROL_MODE_ENABLE)

    # torque_mode alias for torque_control_mode_enable
    torque_mode_bytes = torque_control_mode_enable_bytes
    torque_mode       = torque_control_mode_enable

    # MARK Goal Torque
    goal_torque_bytes = RWByteMotorControl(pt.GOAL_TORQUE)
    goal_torque       =     RWMotorControl(pt.GOAL_TORQUE)


class GoalAccelerationExtra(object):

    # MARK Goal Acceleration
    goal_acceleration_bytes = RWByteMotorControl(pt.GOAL_ACCELERATION)
    goal_acceleration       =     RWMotorControl(pt.GOAL_ACCELERATION)


class AXMotor(Motor, ComplianceMarginSlopeExtra):
    """AX-12 motors"""
    pass

class RXMotor(Motor, ComplianceMarginSlopeExtra):
    """RX-28 and RX-64 motors"""
    pass

class EXMotor(Motor, ComplianceMarginSlopeExtra, SensedCurrentExtra):
    pass

class MXMotor(Motor, PIDExtra, CurrentExtra, GoalAccelerationExtra):
    """MX-28, MX-64 and MX-128 motors"""
    pass

class MX28Motor(MXMotor):
    pass

class MX64Motor(MXMotor):
    pass

class MX106Motor(MXMotor):
    pass

class VXMotor(Motor, PIDExtra):
    """VX-28, and VX-64 motors, used by the V-Rep simulation"""
    pass

MOTOR_MODELS = {
    'AX-12'   : AXMotor,
    'AX-18'   : AXMotor,
    'AX-12W'  : AXMotor,

    'RX-10'   : RXMotor,
    'RX-24F'  : RXMotor,
    'RX-28'   : RXMotor,
    'RX-64'   : RXMotor,

    'MX-28'   : MXMotor,
    'MX-64'   : MX64Motor,
    'MX-106'  : MX106Motor,

    'EX-106+' : EXMotor,

    'VX-28'   : VXMotor,
    'VX-64'   : VXMotor,
}
