import threading
import collections

import protocol
import conversions as conv
import limits
from .. import color

class DynamixelMotor(object):
    """
        This class allows you to control and read the motor values transparently.

        When reading a value, the cached (most up to date) value is returned. For EEPROM and unchanging RAM values (eg compliance slopes), this is not a problem. For speed, pos and load, that means that the values of the last loop of the controller are provided. Depending on the current speed of the motor and the speed of the serial loop, these values might not reflect accurrately the actual status of the motor, but this is the best the hardware is possible to do. Some other RAM values may change quickly and often (torque_enable, led, present_voltage, present_temperature, registered, moving and current (or sensed_current)), and if an updated value is desired, the user should use the request_read() method, with the previous names as argument.

        Writing a value is not immediate, and will be done during the next controller loop. Given a motor m and a variable (eg, torque_enable), you can read the cached value using m.torque_enable, request a new value setting m.torque_enable to a new value : m.torque_enable = False. The requested value can be read with the method m.requested('torque_enable'), and may differ from m.torque_enable until the motor value has been updated on the hardware. If the request have already been processed, requested() will return None. Note that if you request multiple values for the same variable, only the last one at the start of the next controller loop will be taken into account.

        For every variable, there is an alternative raw variable if the unit or value differs (no raw for id, firmware for example). You can use them to access the raw integer value present in the hardware memory register.

        Note that this class should not be instanciated, but only its child (AXMotor, RXMotor, MXMotor), since some methods are not implemented.

        """
    def __init__(self, memory):

        self.mmem = memory

        # this dictionary collect write or read request on part of non standart
        # memory (not pos, speed, load (read) or torque lim, moving speed goal pos (write))
        self.requests = collections.OrderedDict()
        # the dictionary is protected by a semaphore since both Motor and
        # Controller access and modify it. But the semaphore should only be acquired for
        # the duration of atomic actions on the dictionary, and not, for instance, the
        # serial communication related to a request.
        self.request_lock = threading.Semaphore(1)

    def __repr__(self):
        return 'M{}'.format(self.id)

    def __cmp__(self, other):
        if self.id < other.id:
            return -1
        elif self.id == other.id:
            return  0
        else:
            return  1


    # MARK Read/Write request

    aliases_read = {
        'compliant'   : 'torque_enable',
        'voltage'     : 'present_voltage',
        'temperature' : 'present_temperature',
        'position'    : 'present_position',
        'speed'       : 'present_speed',
        'load'        : 'present_load',
        'max_temp'    : 'highest_limit_temperature',
    }

    aliases_write = {
        'compliant'   : 'torque_enable',
        'voltage'     : 'present_voltage',
        'position'    : 'goal_position',
        'speed'       : 'moving_speed',
        'max_temp'    : 'highest_limit_temperature',
    }

    def request_read(name):
        if name in aliases:
            name = aliases[name]
        if name in ['present_position', 'present_speed', 'present_load']:
            return

        self.request_lock.acquire()
        self.request[protocol.REG_ADDRESS(name.upper())] = None
        self.request_lock.release()

    def requested(name):
        """Return raw values for the moment"""

        if name in aliases:
            name = aliases[name]

        self.request_lock.acquire()
        value = self.request.get(protocol.REG_ADDRESS(name.upper()), None)
        self.request_lock.release()
        return value


    # MARK Model specific methods

    # Defining virtual methods to convert speed
    # Supporting new motors idiosyncrasies should be easy

    def raw2_movingdps(self, raw):
        raise NotImplementedError

    def movingdps_2raw(self, dps):
        raise NotImplementedError


    # MARK Mode

    @property
    def mode(self):
        return self.mmem.mode

    @mode.setter
    def mode(self):
        limits.checkoneof('mode', ['wheel', 'joint'], val)
        self.request_lock.acquire()
        self.requests['MODE'] = val
        self.request_lock.release()


    # MARK EEPROM properties

    @property
    def id(self):
        return self.mmem.id

    @id.setter
    def id(self, val):
        limits.checkbounds('id', 0, 254, val)
        self.request_lock.acquire()
        self.requests['ID'] = int(val)
        self.request_lock.release()

    @property
    def model(self):
        return self.mmem.model

    @property
    def model_raw(self):
        return self.mmem[protocol.DXL_MODEL_NUMBER]

    @property
    def modelclass(self):
        return self.mmem.modelclass

    @property
    def version(self):
        return self.mmem.firmware

    # MARK Baudrate

    @property
    def baudrate(self):
        return conv.raw2_baudrate(self.mmem[protocol.DXL_BAUD_RATE], self.modelclass)

    @property
    def baudrate_raw(self):
        return self.mmem[protocol.DXL_BAUD_RATE]

    @baudrate.setter
    def baudrate(self, val):
        self.baudrate_raw = conv.baudrate_2raw(val, self.modelclass)

    @baudrate_raw.setter
    def baudrate_raw(self, val):
        # usually, only value 1, 3, 4, 7, 9, 16, 34, 103, 207, (and 250, 251, 252 for MX) are used
        limits.checkbounds('baudrate', 0, 254, val)
        self.request_lock.acquire()
        self.requests['BAUD_RATE'] = int(val)
        self.request_lock.release()


    # MARK Return Delay Time

    @property
    def return_delay_time(self):
        return conv.raw2_return_delay_time(self.mmem[protocol.DXL_RETURN_DELAY_TIME])

    @property
    def return_delay_time_raw(self):
        return self.mmem[protocol.DXL_RETURN_DELAY_TIME]

    @return_delay_time.setter
    def return_delay_time(self, val):
        self.return_delay_time_raw = conv.return_delay_time_2raw(val)

    @return_delay_time_raw.setter
    def return_delay_time_raw(self):
        limits.checkbounds('return_delay_time', 0, 254, int(val))
        self.request_lock.acquire()
        self.requests['RETURN_DELAY_TIME'] = int(val)
        self.request_lock.release()


    # MARK Angle Limits

    @property
    def cw_angle_limit(self):
        return conv.raw2_deg(self.cw_angle_limit_raw, self.modelclass)

    @property
    def cw_angle_limit_raw(self):
        return self.mmem[protocol.DXL_CW_ANGLE_LIMIT]

    @cw_angle_limit.setter
    def cw_angle_limit(self, val):
        self.cw_angle_limit_raw = self.deg_2raw(val)

    @cw_angle_limit_raw.setter
    def cw_angle_limit_raw(self, val):
        limits.checkbounds('cw_angle_limit', 0, limits.position_range[self.modelclass], int(val))
        self.request_lock.acquire()
        self.requests['CW_ANGLE_LIMIT'] = int(val)
        self.request_lock.release()

    @property
    def ccw_angle_limit(self):
        return conv.raw2_deg(self.ccw_angle_limit_raw, self.modelclass)

    @property
    def ccw_angle_limit_raw(self):
        return self.mmem[protocol.DXL_CCW_ANGLE_LIMIT]

    @ccw_angle_limit.setter
    def ccw_angle_limit(self, val):
        self.ccw_angle_limit_raw = self.deg_2raw(val)

    @ccw_angle_limit_raw.setter
    def ccw_angle_limit_raw(self, val):
        limits.checkbounds('ccw_angle_limit', 0, limits.position_range[self.modelclass], int(val))
        self.request_lock.acquire()
        self.requests['CCW_ANGLE_LIMIT'] = int(val)
        self.request_lock.release()


    @property
    def angle_limits(self):
        return self.cw_angle_limit, self.ccw_angle_limit

    @property
    def angle_limits_raw(self):
        return self.cw_angle_limit_raw, self.ccw_angle_limit_raw

    @angle_limits.setter
    def angle_limits(self, val):
        self.angle_limits_raw = self.deg_2raw(val[0]), self.deg_2raw(val[1])

    @angle_limits_raw.setter
    def angle_limits_raw(self, val):
        if int(val[0]) > int(val[1]):
            raise ValueError('CW angle limit ({}) should be inferior to CCW angle limit ({})'.format(val[0], val[1]))
        limits.checkbounds('cw_angle_limit', 0, limits.position_range[self.modelclass], int(val[0]))
        limits.checkbounds('ccw_angle_limit', 0, limits.position_range[self.modelclass], int(val[1]))
        self.request_lock.acquire()
        self.requests['CCW_ANGLE_LIMIT'] = int(val[0]), int(val[1])
        self.request_lock.release()


    # MARK Highest Limit Temperature

    @property
    def highest_limit_temperature(self):
        return self.highest_limit_temperature_raw

    @property
    def highest_limit_temperature_raw(self):
        return self.mmem[protocol.DXL_HIGHEST_LIMIT_TEMPERATURE]

    @highest_limit_temperature.setter
    def highest_limit_temperature(self, val):
        self.highest_limit_temperature_raw = int(val)

    @highest_limit_temperature_raw.setter
    def highest_limit_temperature_raw(self, val):

        limits.checkbounds('highest_limit_temperature', 10, 99, int(val))
        self.request_lock.acquire()
        self.requests['HIGHEST_LIMIT_TEMPERATURE'] = int(val)
        self.request_lock.release()


    # MARK Highest Limit Voltage

    @property
    def highest_limit_voltage(self):
        return conv.raw2_voltage(self.highest_limit_voltage_raw)

    @property
    def highest_limit_voltage_raw(self):
        return self.mmem[protocol.DXL_HIGHEST_LIMIT_VOLTAGE]

    @highest_limit_voltage.setter
    def highest_limit_voltage(self, val):
        self.highest_voltage_raw = int(val)

    @highest_limit_voltage_raw.setter
    def highest_limit_voltage_raw(self, val):

        limits.checkbounds('highest_limit_voltage', 10, 99, int(val))
        self.request_lock.acquire()
        self.requests['HIGHEST_LIMIT_VOLTAGE'] = int(val)
        self.request_lock.release()


    # MARK Lowest Limit Voltage

    @property
    def lowest_limit_voltage(self):
        return conv.raw2_voltage(self.lowest_limit_voltage_raw)

    @property
    def lowest_limit_voltage_raw(self):
        return self.mmem[protocol.DXL_LOWEST_LIMIT_VOLTAGE]

    @lowest_limit_voltage.setter
    def lowest_limit_voltage(self, val):
        self.lowest_voltage_raw = int(val)

    @lowest_limit_voltage_raw.setter
    def lowest_limit_voltage_raw(self, val):

        limits.checkbounds('lowest_limit_voltage', 10, 99, int(val))
        self.request_lock.acquire()
        self.requests['LOWEST_LIMIT_VOLTAGE'] = int(val)
        self.request_lock.release()


    # max_temp is provided as a conveniance alias

    @property
    def max_temp(self):
        return self.highest_limit_temperature

    @property
    def max_temp_raw(self):
        return self.highest_limit_temperature_raw

    @max_temp.setter
    def max_temp(self, val):
        self.highest_limit_temperature_raw = int(val)

    @max_temp_raw.setter
    def max_temp_raw(self, val):
        self.highest_limit_temperature_raw = val


    # MARK Limit Voltages (TODO: setters)

    @property
    def min_voltage(self):
        return conv.raw2_voltage(self.mmem[protocol.DXL_LOWEST_LIMIT_VOLTAGE])

    @property
    def min_voltage_raw(self):
        return self.mmem[protocol.DXL_LOWEST_LIMIT_VOLTAGE]

    @property
    def max_voltage(self):
        return conv.raw2voltage(self.mmem[protocol.DXL_HIGEST_LIMIT_VOLTAGE])

    @property
    def max_voltage_raw(self):
        return self.mmem[protocol.DXL_HIGEST_LIMIT_VOLTAGE]


    # MARK Max Torque

    @property
    def max_torque(self):
        return conv.raw2_torque(self.mmem[protocol.DXL_MAX_TORQUE])

    @property
    def max_torque_raw(self):
        return self.mmem[protocol.DXL_MAX_TORQUE]

    @max_torque.setter
    def max_torque(self, val):
        self.max_torque_raw = conv.torque_2raw(val)

    @max_torque_raw.setter
    def max_torque_raw(self, val):
        limits.checkbounds('max_torque', 0, 1023, int(val))
        self.request_lock.acquire()
        self.requests['MAX_TORQUE'] = int(val)
        self.request_lock.release()


    # MARK Return Status Level

    @property
    def status_return_level(self):
        return self.mmem.status_return_level

    @status_return_level.setter
    def status_return_level(self, val):
        limits.checkoneof('compliant', [0, 1, 2], int(val))
        self.request_lock.acquire()
        self.requests['RETURN_STATUS_LEVEL'] = int(val)
        self.request_lock.release()



    # MARK RAM properties

    # MARK Torque Enable

    @property
    def torque_enable(self):
        return bool(self.mmem[protocol.DXL_TORQUE_ENABLE])

    @property
    def torque_enable_raw(self):
        return self.mmem[protocol.DXL_TORQUE_ENABLE]

    @torque_enable.setter
    def torque_enable(self, val):
        self.torque_enable_raw = int(val)

    @torque_enable_raw.setter
    def torque_enable_raw(self, val):
        limits.checkoneof('torque_enable', [0, 1], int(val))
        self.requests['TORQUE_ENABLE'] = int(val)


    # compliant is a more intuive alternative for torque_enable
    # Beware ! compliant == not torque_enable. (for this reason, compliant_raw
    # does not exists)

    @property
    def compliant(self):
        return not self.torque_enable

    @compliant.setter
    def compliant(self, val):
        self.torque_enable = not val


    # MARK LED

    @property
    def led(self):
        return bool(self.led_raw)

    @property
    def led_raw(self):
        return self.mmem[protocol.DXL_LED]

    @led.setter
    def led(self, val):
        self.led_raw = int(val)

    @led_raw.setter
    def led_raw(self, val):
        """Changing the goal position will turn on torque_enable if off."""
        limits.checkoneof('led', [0, 1], int(val))
        self.request_lock.acquire()
        self.requests['LED'] = int(val)
        self.request_lock.release()


    # MARK Goal position

    @property
    def goal_position(self):
        return conv.raw2_deg(self.goal_position_raw, self.modelclass)

    @property
    def goal_position_raw(self):
        return self.mmem[protocol.DXL_GOAL_POSITION]

    @goal_position.setter
    def goal_position(self, val):
        self.goal_position_raw = conv.deg_2raw(val, self.modelclass)

    @goal_position_raw.setter
    def goal_position_raw(self, val):
        """Changing the goal position will turn on torque_enable if off."""
        limits.checkbounds('goal_position',
                         self.cw_angle_limit_raw, self.ccw_angle_limit_raw, int(val))
        self.request_lock.acquire()
        self.requests['GOAL_POSITION'] = int(val)
        self.request_lock.release()


    # MARK moving_speed

    @property
    def moving_speed(self):
        return self.raw2_movingdps(self.moving_speed_raw)

    @property
    def moving_speed_raw(self):
        return self.mmem[protocol.DXL_MOVING_SPEED]

    @moving_speed.setter
    def moving_speed(self, val):
        self.moving_speed_raw = self.movingdps_2raw(val)

    @moving_speed_raw.setter
    def moving_speed_raw(self, val):
        if self.mode == 'wheel':
            limits.checkbounds_mode('moving_speed', 0, 2047, int(val), self.mode)
        else:
            limits.checkbounds_mode('moving_speed', 0, 1023, int(val), self.mode)
        self.request_lock.acquire()
        self.requests['MOVING_SPEED'] = int(val)
        self.request_lock.release()


    # MARK torque_limit

    @property
    def torque_limit(self):
        return self.raw2_torque(self.torque_limit_raw)

    @property
    def torque_limit_raw(self):
        return self.mmem[protocol.DXL_TORQUE_LIMIT]

    @torque_limit.setter
    def torque_limit(self, val):
        self.torque_limit_raw = conv.deg_2raw(val, self.modelclass)

    @torque_limit_raw.setter
    def torque_limit_raw(self, val):

        limits.checkbounds_mode('torque_limit', 0, 1023, int(val))
        self.request_lock.acquire()
        self.requests['TORQUE_LIMIT'] = int(val)
        self.request_lock.release()


    # MARK present_position, present_speed, present_load

    @property
    def present_position(self):
        return conv.raw2_deg(self.present_position_raw, self.modelclass)

    @property
    def present_position_raw(self):
        return self.mmem[protocol.DXL_PRESENT_POSITION]

    @property
    def present_speed(self):
        return conv.raw2_cwccwdps(self.present_speed_raw, self.modelclass)

    @property
    def present_speed_raw(self):
        return self.mmem[protocol.DXL_PRESENT_SPEED]

    @property
    def present_load(self):
        return conv.raw2_load(self.mmem[protocol.DXL_PRESENT_LOAD])

    @property
    def present_load_raw(self):
        return self.mmem[protocol.DXL_PRESENT_LOAD]


    # MARK position, speed

    # Here you can read and set position and speed through only two attribute.
    # You're missing out on timing subtelty, and it may lead to unexpected
    # behavior. But in most case, you're probably fine.

    @property
    def position(self):
        return self.present_position

    @property
    def position_raw(self):
        return self.present_position_raw

    @position.setter
    def position(self, val):
        self.goal_position = val

    @position_raw.setter
    def position_raw(self, val):
        self.goal_position_raw = val

    @property
    def speed(self):
        return self.present_speed

    @property
    def speed_raw(self):
        return self.present_speed_raw

    @speed.setter
    def speed(self, val):
        self.moving_speed = val

    @speed_raw.setter
    def speed_raw(self, val):
        self.moving_speed_raw = val


    # MARK Present voltage

    @property
    def present_voltage(self):
        return conv.raw2_voltage(self.present_voltage_raw)

    @property
    def present_voltage_raw(self):
        return self.mmem[protocol.DXL_PRESENT_VOLTAGE]

    @property
    def voltage(self):
        return self.present_voltage

    @property
    def voltage_raw(self):
        return self.present_voltage_raw


    # MARK Present temperature

    @property
    def present_temperature(self):
        return conv.raw2_voltage(self.present_temperature_raw)

    @property
    def present_temperature_raw(self):
        return self.mmem[protocol.DXL_PRESENT_TEMPERATURE]

    @property
    def temperature(self):
        return self.present_voltage

    @property
    def temperature_raw(self):
        return self.present_voltage_raw


    # MARK Registered

    @property
    def registered(self):
        return bool(self.registered_raw)

    @property
    def registered_raw(self):
        return self.mmem[protocol.DXL_REGISTERED]


    # MARK Lock EEPROM

    @property
    def lock(self):
        return bool(self.registered_raw)

    @property
    def lock_raw(self):
        return self.mmem[protocol.DXL_LOCK]

    # TODO Locking


    # MARK Moving

    @property
    def moving(self):
        return bool(self.moving_raw)

    @property
    def moving_raw(self):
        return self.mmem[protocol.DXL_MOVING]


    # MARK Punch

    @property
    def punch(self):
        return conv.raw2_torque(self.punch_raw)

    @property
    def punch_raw(self):
        return self.mmem[protocol.DXL_MOVING]

    @punch.setter
    def punch(self, val):
        self.punch_raw = conv.torque_2raw(val)

    @punch_raw.setter
    def punch_raw(self, val):
        limits.checkbounds_mode('punch', 32, 1023, int(val))
        self.request_lock.acquire()
        self.requests['PUNCH'] = int(val)
        self.request_lock.release()


class AXRXEXMotor(DynamixelMotor):

    # MARK Conversion methods

    def raw2_movingdps(self, raw):
        if self.mode == 'wheel':
            return conv.raw2_torquespeed(raw)
        else:
            return conv.raw2_movingdps(raw, self.modelclass)

    def movingdps_2raw(self, dps):
        if self.mode == 'wheel':
            return conv.torquespeed_2raw(dps)
        else:
            return conv.movingdps_2raw(dps, self.modelclass)


    # MARK Compliance margin

    # Compliance margins are aggressively cached since they are not changed
    # other than through user intervention

    @property
    def cw_compliance_margin(self):
        return raw2_deg(self.mmem[protocol.DXL_CW_COMPLIANCE_MARGIN])

    @property
    def cw_compliance_margin_raw(self):
        return self.mmem[protocol.DXL_CW_COMPLIANCE_MARGIN]


    @cw_compliance_margin.setter
    def cw_compliance_margin(self, val):
        self.cw_compliance_margin_raw = val

    @cw_compliance_margin_raw.setter
    def cw_compliance_margin_raw(self, val):
        limits.checkbounds('cw compliance margin', 0, 255, int(val))
        self.request_lock.acquire()
        self.requests['CW_COMPLIANCE_MARGIN'] = int(val)
        self.request_lock.release()


    @property
    def ccw_compliance_margin(self):
        return self.raw2_deg(self.mmem[protocol.DXL_CCW_COMPLIANCE_MARGIN])

    @property
    def ccw_compliance_margin_raw(self):
        return self.mmem[protocol.DXL_CCW_COMPLIANCE_MARGIN]

    @ccw_compliance_margin.setter
    def ccw_compliance_margin(self, val):
        self.ccw_compliance_margin_raw = val

    @ccw_compliance_margin_raw.setter
    def ccw_compliance_margin_raw(self, val):
        limits.checkbounds('ccw compliance margin', 0, 255, int(val))
        self.request_lock.acquire()
        self.requests['CCW_COMPLIANCE_MARGIN'] = int(val)
        self.request_lock.release()


    @property
    def compliance_margins(self):
        return self.cw_compliance_margin, self.ccw_compliance_margin

    @property
    def compliance_margins_raw(self):
        return self.cw_compliance_margin_raw, self.ccw_compliance_margin_raw

    # here we can do only one write
    @compliance_margins.setter
    def compliance_margins(self, val):
        self.compliance_margins_raw = val

    @compliance_margins_raw.setter
    def compliance_margin_raw(self, val):
        limits.checkbounds('ccw compliance margin', 0, 255, int(val[0]))
        limits.checkbounds('ccw compliance margin', 0, 255, int(val[1]))
        self.request_lock.acquire()
        self.requests['COMPLIANCE_MARGINS'] = int(val[0]), int(val[1])
        self.request_lock.release()


    # MARK Printing EEPROM, RAM

    def _mem_desc(self, addr, desc):
        return ('{}{:2d}{}'.format(color.cyan, addr, color.end),
                '{}{:4d}  {}{}{}'.format(color.purple, self.mmem[addr], color.grey, desc, color.end))
        
    def eeprom_desc(self):
        s = ['EEPROM', 
             '{} Model                 : {}'.format(*self._mem_desc(0, self.model)),
             '{} Firware               : {}'.format(*self._mem_desc(2, '')),
             '{} ID                    : {}'.format(*self._mem_desc(3, '')),
             '{} Baud Rate             : {}'.format(*self._mem_desc(4, '{} bauds'.format(self.baudrate))),
             '{} Return Delay Time     : {}'.format(*self._mem_desc(5, '{} usec'.format(self.return_delay_time))),
             '{} CW Angle Limit        : {}'.format(*self._mem_desc(6, '{:6.2f} degrees'.format(self.cw_angle_limit))),
             '{} CCW Angle Limit       : {}'.format(*self._mem_desc(8, '{:6.2f} degrees, {} mode'.format(self.ccw_angle_limit, self.mode))),
             '{} Max Limit Temp        : {}'.format(*self._mem_desc(11, '{} celsius'.format(self.highest_limit_temperature))),
             '{} Min Limit Voltage     : {}'.format(*self._mem_desc(12, '{:4.1f} V'.format(self.lowest_limit_voltage))),
             '{} Max Limit Voltage     : {}'.format(*self._mem_desc(13, '{:4.1f} V'.format(self.highest_limit_voltage))),
             '{} Max Torque            : {}'.format(*self._mem_desc(14, '{:.1f} % of max'.format(self.max_torque))),
             '{} Status Return Level   : {}'.format(*self._mem_desc(16, '')),
             #'{} Alarm LED             : {}' % column(17, aled),
             #'{} Alarm Shutdown        : {}' % column(18, ashutdown),
             '\n'+color.end]
        s = '\n'.join(s)
        return s



class AXMotor(AXRXEXMotor):
    pass

class RXMotor(AXRXEXMotor):
    pass

class EXMotor(AXRXEXMotor):

    @property
    def sensed_current(self):
        return self.raw2_sensed_current(self.sensed_current_raw)

    @property
    def sensed_current_raw(self):
        return self.mmem[protocol.DXL_SENSED_CURRENT]




class MXMotor(DynamixelMotor):

    # MARK Conversion methods

    def raw2_movingdps(self, raw):
        return conv.raw2_movingdps(raw, self.modelclass)

    def movingdps_2raw(self, dps):
        return conv.movingdps_2raw(dps, self.modelclass)



    # MARK PID Gains

    # PID gains are aggressively cached since they are not changed
    # other than through user intervention

    @property
    def p_gain(self):
        return conv.raw2_pgain(self.p_gain_raw)

    @property
    def p_gain_raw(self):
        return self.mmem[protocol.DXL_P_GAIN]

    @p_gain.setter
    def p_gain(self, val):
        self.p_gain_raw = conv.pgain_2raw(val)

    @p_gain_raw.setter
    def p_gain_raw(self, val):
        limits.checkbounds('p_gain', 0, 254, int(val))
        self.request_lock.acquire()
        self.requests['P_GAIN'] = int(val)
        self.request_lock.release()

    @property
    def i_gain(self):
        return conv.raw2_igain(self.i_gain_raw)

    @property
    def i_gain_raw(self):
        return self.mmem[protocol.DXL_I_GAIN]

    @i_gain.setter
    def i_gain(self, val):
        self.i_gain_raw = conv.igain_2raw(val)

    @i_gain_raw.setter
    def i_gain_raw(self, val):
        limits.checkbounds('i_gain', 0, 254, int(val))
        self.request_lock.acquire()
        self.requests['I_GAIN'] = int(val)
        self.request_lock.release()

    @property
    def d_gain(self):
        return conv.raw2_dgain(self.d_gain_raw)

    @property
    def d_gain_raw(self):
        return self.mmem[protocol.DXL_D_GAIN]

    @d_gain.setter
    def d_gain(self, val):
        self.d_gain_raw = conv.dgain_2raw(val)

    @d_gain_raw.setter
    def d_gain_raw(self, val):
        limits.checkbounds('d_gain', 0, 254, int(val))
        self.request_lock.acquire()
        self.requests['D_GAIN'] = int(val)
        self.request_lock.release()

    @property
    def gains(self):
        return conv.raw2_gains(self.gains_raw)

    @property
    def gains_raw(self):
        return (self.mmem[protocol.DXL_D_GAIN],
                self.mmem[protocol.DXL_I_GAIN],
                self.mmem[protocol.DXL_P_GAIN])

    @gains.setter
    def gains(self, val):
        self.gains_raw = conv.gains_2raw(val)

    @gains_raw.setter
    def gains_raw(self, val):
        limits.checkbounds('d_gain', 0, 254, int(val[0]))
        limits.checkbounds('i_gain', 0, 254, int(val[1]))
        limits.checkbounds('p_gain', 0, 254, int(val[2]))
        self.request_lock.acquire()
        self.requests['GAINS'] = (int(val[0]), int(val[1]), int(val[2]))
        self.request_lock.release()

    # MARK Current

    @property
    def current(self):
        return conv.raw2_current(self.current_raw)
    
    @property
    def current_raw(self):
        return self.mmem[protocol.DXL_CURRENT]
    
    @current.setter
    def current(self, val):
        self.current_raw = conv.dgain_2raw(val)
    
    @current_raw.setter
    def current_raw(self, val):
        limits.checkbounds('current raw', 0, 4095, int(val))
        self.request_lock.acquire()
        self.requests['CURRENT'] = int(val)
        self.request_lock.release()

# Only the MX64 and 106 seems to support current.
# (although you have to go through the korean doc for the MX64 to know that)

class MX64Motor(MXMotor):

    # MARK Torque Control Mode

    @property
    def torque_control_mode_enable(self):
        return bool(self.torque_control_mode_enable_raw)

    @property
    def torque_control_mode_enable_raw(self):
        return self.mmem[protocol.DXL_TORQUE_CONTROL_MODE_ENABLE]

    @torque_control_mode_enable.setter
    def torque_control_mode_enable(self, val):
        self.torque_control_mode_enable_raw = conv.dgain_2raw(val)

    @torque_control_mode_enable_raw.setter
    def torque_control_mode_enable_raw(self, val):
        limits.checkoneof('torque_control_mode_enable raw', [0, 1], int(val))
        self.request_lock.acquire()
        self.requests['TORQUE_CONTROL_MODE_ENABLE'] = int(val)
        self.request_lock.release()

    # torque_mode alias for torque_control_mode_enable

    @property
    def torque_mode(self):
        return self.torque_control_enable
    
    @property
    def torque_mode_raw(self):
        return self.torque_control_enable_raw
    
    @torque_mode.setter
    def torque_mode(self, val):
        self.torque_control_enable = val
    
    @torque_mode_raw.setter
    def torque_mode_raw(self, val):
        self.torque_control_enable_raw = val


    # MARK Goal Torque

    @property
    def goal_torque(self):
        return conv.raw2_goaltorque(self.goal_torque_raw)

    @property
    def goal_torque_raw(self):
        return self.mmem[protocol.DXL_GOAL_TORQUE]

    @goal_torque.setter
    def goal_torque(self, val):
        self.goal_torque_raw = conv.goaltorque_2raw(val)
    
    @goal_torque_raw.setter
    def goal_torque_raw(self):
        limits.checkbounds('goal torque raw', 0, 2047, int(val))
        self.request_lock.acquire()
        self.requests['GOAL_TORQUE'] = int(val)
        self.request_lock.release()


    # MARK Goal Acceleration
    
    @property
    def goal_acceleration(self):
        return conv.raw2_goalacc(self.goal_acceleration_raw)
    
    @property
    def goal_acceleration_raw(self):
        return self.mmem[protocol.DXL_GOAL_ACCELERATION]
    
    @goal_acceleration.setter
    def goal_acceleration(self, val):
        self.goal_acceleration_raw = conv.goalacc_2raw(val)
    
    @goal_acceleration_raw.setter
    def goal_acceleration_raw(self):
        limits.checkbounds('goal acceleration raw', 0, 254, int(val))
        self.request_lock.acquire()
        self.requests['GOAL_ACCELERATION'] = int(val)
        self.request_lock.release()


class MX106Motor(MX64Motor):

    pass