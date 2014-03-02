"""
Conversion modules

Host all the conversion fonctions we need to make motor values more human.
Conversion fonctions follow the naming convention:

    def control_name_2bytes(value, modelclass=None, mode=None)
    def bytes2_control_name(value, modelclass=None, mode=None)

which convert human values to bytes and bytes to human values respectively.
Note that two-bytes values are returned as one integer:



Some functions require the motor modelclass ('AX', 'RX', 'MX', ...) and mode
('joint' or 'wheel') to function properly. For consistency, all conversions
function accept thoses arguments.

All function are accessible through the conversions.CONV dictionnary:

    CONV[pt.MAX_TORQUE] = (max_torque_2bytes, bytes2_max_torque)

Functions do check that arguments are in their legal ranges.
"""

#pylint: disable=C0103,W0613,C0326

from __future__ import division

from ..refs import protocol as pt
from ..refs import limits
from . import alarms

# for details, see http://support.robotis.com/en/product/dynamixel/

CONV = {}

# MARK For simple cases

def _def_bytes2_onebyte(control):
    def _b2b(value, modelclass=None, mode=None):
        limits.CHECK_BYTES[control](value)
        return int(value)
    return _b2b

def _def_onebyte_2bytes(control):
    def _b2b(value, modelclass=None, mode=None):
        limits.CHECK[control](value)
        return int(value)
    return _b2b


# MARK Model Number

MODEL_NAMES = {
    113: 'DX-113',
    116: 'DX-116',
    117: 'DX-117',

    12:  'AX-12',
    18:  'AX-18',
    44:  'AX-12W',

    10:  'RX-10',
    24:  'RX-24F',
    28:  'RX-28',
    64:  'RX-64',

    360: 'MX-12',
    29:  'MX-28',
    54:  'MX-64',
    320: 'MX-106',

    107: 'EX-106+',

    10028: 'VX-28', # VREP
    10064: 'VX-64', # VREP
}
MODEL_NUMBERS = {name: number for number, name in MODEL_NAMES.items()}


def bytes2_model_number(value, modelclass=None, mode=None):
    limits.CHECK_BYTES[pt.MODEL_NUMBER](value)
    return MODEL_NAMES[value]

def model_number_2bytes(value, modelclass=None, mode=None):
    limits.CHECK[pt.MODEL_NUMBER](value)
    return MODEL_NUMBERS[value.upper()]

CONV[pt.MODEL_NUMBER] = (model_number_2bytes, bytes2_model_number)


# MARK Firmware

bytes2_firmware = _def_bytes2_onebyte(pt.FIRMWARE)
firmware_2bytes = _def_onebyte_2bytes(pt.FIRMWARE)
CONV[pt.FIRMWARE] = (firmware_2bytes, bytes2_firmware)


# MARK ID

bytes2_id = _def_bytes2_onebyte(pt.FIRMWARE)
id_2bytes = _def_onebyte_2bytes(pt.FIRMWARE)
CONV[pt.ID] = (id_2bytes, bytes2_id)


# MARK Status Return Level

bytes2_status_return_level = _def_bytes2_onebyte(pt.STATUS_RETURN_LEVEL)
status_return_level_2bytes = _def_onebyte_2bytes(pt.STATUS_RETURN_LEVEL)
CONV[pt.STATUS_RETURN_LEVEL] = (status_return_level_2bytes, bytes2_status_return_level)


# MARK Baudrate

BAUDRATE_MX = {
    250 : 2250000,
    251 : 2500000,
    252 : 3000000,
}

MX_BAUDRATE = {value:key for key, value in BAUDRATE_MX.items()}


def bytes2_baudrate(value, modelclass=None, mode=None):
    limits.CHECK_BYTES[pt.BAUDRATE](value)
    if modelclass == 'MX':
        try:
            return BAUDRATE_MX[value]
        except KeyError:
            pass
    return 2000000.0/(value + 1)


def baudrate_2bytes(value, modelclass=None, mode=None):
    limits.CHECK[pt.BAUDRATE](value, modelclass)
    if modelclass == 'MX':
        try:
            return MX_BAUDRATE[value]
        except KeyError:
            pass
    return int(2000000.0/value - 1)

CONV[pt.BAUDRATE] = (baudrate_2bytes, bytes2_baudrate)


# MARK Return delay time

def return_delay_time_2bytes(value, modelclass=None, mode=None):
    """in microseconds"""
    limits.CHECK[pt.RETURN_DELAY_TIME](value)
    return int(value/2)

def bytes2_return_delay_time(value, modelclass=None, mode=None):
    """in microseconds"""
    limits.CHECK_BYTES[pt.RETURN_DELAY_TIME](value)
    return 2*value

CONV[pt.RETURN_DELAY_TIME] = (return_delay_time_2bytes, bytes2_return_delay_time)


# MARK Temperature

bytes2_highest_limit_temperature = _def_bytes2_onebyte(pt.HIGHEST_LIMIT_TEMPERATURE)
highest_limit_temperature_2bytes = _def_onebyte_2bytes(pt.HIGHEST_LIMIT_TEMPERATURE)
CONV[pt.HIGHEST_LIMIT_TEMPERATURE] = (highest_limit_temperature_2bytes, bytes2_highest_limit_temperature)

bytes2_present_temperature = _def_bytes2_onebyte(pt.PRESENT_TEMPERATURE)
present_temperature_2bytes = _def_onebyte_2bytes(pt.PRESENT_TEMPERATURE)
CONV[pt.PRESENT_TEMPERATURE] = (present_temperature_2bytes, bytes2_present_temperature)


# MARK Voltage

def _def_bytes2_voltage(control):
    def _b2b(value, modelclass=None, mode=None):
        """Return the voltage in volt"""
        limits.CHECK_BYTES[control](value)
        return value/10.0
    return _b2b

def _def_voltage_2bytes(control):
    def _b2b(value, modelclass=None, mode=None):
        """Return the voltage in volt"""
        limits.CHECK[control](value)
        return int(10.0*value)
    return _b2b

bytes2_highest_limit_voltage = _def_bytes2_voltage(pt.HIGHEST_LIMIT_VOLTAGE)
highest_limit_voltage_2bytes = _def_voltage_2bytes(pt.HIGHEST_LIMIT_VOLTAGE)
CONV[pt.HIGHEST_LIMIT_VOLTAGE] = (highest_limit_voltage_2bytes, bytes2_highest_limit_voltage)

bytes2_lowest_limit_voltage = _def_bytes2_voltage(pt.LOWEST_LIMIT_VOLTAGE)
lowest_limit_voltage_2bytes = _def_voltage_2bytes(pt.LOWEST_LIMIT_VOLTAGE)
CONV[pt.LOWEST_LIMIT_VOLTAGE] = (lowest_limit_voltage_2bytes, bytes2_lowest_limit_voltage)

bytes2_present_voltage = _def_bytes2_voltage(pt.PRESENT_VOLTAGE)
present_voltage_2bytes = _def_voltage_2bytes(pt.PRESENT_VOLTAGE)
CONV[pt.PRESENT_VOLTAGE] = (present_voltage_2bytes, bytes2_present_voltage)


# MARK Torque

def _def_bytes2_percent(control):
    def _b2b(value, modelclass=None, mode=None):
        """Map [0, 1023] to [0%, 100%]"""
        limits.CHECK_BYTES[control](value)
        return 100.0*value/1023.0
    return _b2b

def _def_percent_2bytes(control):
    def _b2b(value, modelclass=None, mode=None):
        limits.CHECK[control](value)
        return int(value/100.0*1023)
    return _b2b

max_torque_2bytes = _def_percent_2bytes(pt.MAX_TORQUE)
bytes2_max_torque = _def_bytes2_percent(pt.MAX_TORQUE)
CONV[pt.MAX_TORQUE] = (max_torque_2bytes, bytes2_max_torque)

torque_limit_2bytes = _def_percent_2bytes(pt.TORQUE_LIMIT)
bytes2_torque_limit = _def_bytes2_percent(pt.TORQUE_LIMIT)
CONV[pt.TORQUE_LIMIT] = (torque_limit_2bytes, bytes2_torque_limit)

punch_2bytes = _def_percent_2bytes(pt.PUNCH)
bytes2_punch = _def_bytes2_percent(pt.PUNCH)
CONV[pt.PUNCH] = (punch_2bytes, bytes2_punch)


# MARK Position

def _def_bytes2_degree(control):
    def _b2b(value, modelclass=None, mode=None):
        limits.CHECK_BYTES[control](value, modelclass=modelclass, mode=mode)
        max_pos, max_deg = limits.POSITION_RANGES[modelclass]
        return (value / max_pos - 0.5) * max_deg
    return _b2b

def _def_degree_2bytes(control):
    def _b2b(value, modelclass=None, mode=None):
        limits.CHECK[control](value, modelclass=modelclass, mode=mode)
        max_pos, max_deg = limits.POSITION_RANGES[modelclass]
        return int(round((value / max_deg + 0.5) * max_pos))
    return _b2b

# def safedeg_2bytes(value, modelclass=None, mode=None):
#     min_angle = bytes2_deg(mmem[pt.CW_ANGLE_LIMIT], modelclass=modelclass, mode=mode)
#     max_angle = bytes2_deg(mmem[pt.CCW_ANGLE_LIMIT], modelclass=modelclass, mode=mode)
#     limits.checkbounds('safe position', min_angle - 0.001, max_angle + 0.001, value)
#     return deg_2bytes(value, modelclass=None, mode=None)


bytes2_position = _def_bytes2_degree(pt.PRESENT_POSITION)
position_2bytes = _def_degree_2bytes(pt.GOAL_POSITION)

bytes2_goal_position = _def_bytes2_degree(pt.GOAL_POSITION)
goal_position_2bytes = _def_degree_2bytes(pt.GOAL_POSITION)
CONV[pt.GOAL_POSITION] = (goal_position_2bytes, bytes2_goal_position)

bytes2_present_position = _def_bytes2_degree(pt.PRESENT_POSITION)
present_position_2bytes = _def_degree_2bytes(pt.PRESENT_POSITION) # useless
CONV[pt.PRESENT_POSITION] = (present_position_2bytes, bytes2_present_position)

bytes2_cw_angle_limit   = _def_bytes2_degree(pt.CW_ANGLE_LIMIT)
cw_angle_limit_2bytes   = _def_degree_2bytes(pt.CW_ANGLE_LIMIT)
CONV[pt.CW_ANGLE_LIMIT] = (cw_angle_limit_2bytes, bytes2_cw_angle_limit)

bytes2_ccw_angle_limit  = _def_bytes2_degree(pt.CCW_ANGLE_LIMIT)
ccw_angle_limit_2bytes  = _def_degree_2bytes(pt.CCW_ANGLE_LIMIT)
CONV[pt.CCW_ANGLE_LIMIT] = (ccw_angle_limit_2bytes, bytes2_ccw_angle_limit)


# MARK Speed

SPEED_RATIOS = {
    'AX': 6*0.111,
    'DX': 6*0.111,
    'RX': 6*0.111,
    'EX': 6*0.111,
    'MX': 6*0.11445,
    'VX': 6*0.111,
}

def bytes2_present_speed(value, modelclass=None, mode=None):
    """
    present speed conversion, in degree per second and percent.
    For model other than MX, in wheel mode, present speed return
    the amount of torque exerced on the motor.
    """
    limits.CHECK_BYTES[pt.PRESENT_SPEED](value, modelclass=modelclass, mode=mode)
    if modelclass != 'MX' and mode == 'wheel':
        return bytes2_present_load(value)
    else:
        direction = ((value >> 10) * 2) - 1
        return direction * (value % 1024) * SPEED_RATIOS[modelclass]

def present_speed_2bytes(value, modelclass=None, mode=None):
    """
    present speed conversion, in degree per second and percent.
    For model other than MX, in wheel mode, present speed return
    the amount of torque exerced on the motor.
    """
    limits.CHECK[pt.PRESENT_SPEED](value, modelclass=modelclass, mode=mode)
    if modelclass != 'MX' and mode == 'wheel':
        return present_load_2bytes(value)
    else:
        b = int(round(abs(value)/SPEED_RATIOS[modelclass]))
        if value >= 0:
            return b + 1024
        return b

CONV[pt.PRESENT_SPEED] = (present_speed_2bytes, bytes2_present_speed)



def bytes2_moving_speed(value, modelclass=None, mode=None):
    """
    present speed conversion, in degree per second and percent.
    For model other than MX, in wheel mode, present speed return
    the amount of torque exerced on the motor.
    """
    limits.CHECK_BYTES[pt.MOVING_SPEED](value, modelclass=modelclass, mode=mode)
    if mode == 'joint':
        return value*SPEED_RATIOS[modelclass]
    else: # wheel mode
        return bytes2_present_speed(value, modelclass=modelclass, mode=mode)

def moving_speed_2bytes(value, modelclass=None, mode=None):
    """
    present speed conversion, in degree per second and percent.
    For model other than MX, in wheel mode, present speed return
    the amount of torque exerced on the motor.
    """
    limits.CHECK[pt.MOVING_SPEED](value, modelclass=modelclass, mode=mode)
    if mode == 'joint':
        return int(round(value/SPEED_RATIOS[modelclass]))
    else:
        return present_speed_2bytes(value, modelclass=modelclass, mode=mode)

CONV[pt.MOVING_SPEED] = (moving_speed_2bytes, bytes2_moving_speed)


# MARK Load

def bytes2_present_load(value, modelclass=None, mode=None):
    """return the load into signed torque percent"""
    limits.CHECK_BYTES[pt.PRESENT_LOAD](value)
    direction = ((value >> 10) * 2) - 1

    return direction * (value % 1024) * 0.1

def present_load_2bytes(value, modelclass=None, mode=None):
    limits.CHECK[pt.PRESENT_LOAD](value)
    if value > 0:
        return int(round( value*1023/100.0))+1024
    else:
        return int(round(-value*1023/100.0))

CONV[pt.PRESENT_LOAD] = (present_load_2bytes, bytes2_present_load)


# MARK: - Gain conversions

def bytes2_d_gain(value, modelclass=None, mode=None):
    limits.CHECK_BYTES[pt.D_GAIN](value)
    return 0.004 * value

def bytes2_i_gain(value, modelclass=None, mode=None):
    limits.CHECK_BYTES[pt.I_GAIN](value)
    return value / 2.048

def bytes2_p_gain(value, modelclass=None, mode=None):
    limits.CHECK_BYTES[pt.P_GAIN](value)
    return 0.125 * value

def d_gain_2bytes(value, modelclass=None, mode=None):
    limits.CHECK[pt.D_GAIN](value)
    return int(250.0 * value)

def i_gain_2bytes(value, modelclass=None, mode=None):
    limits.CHECK[pt.I_GAIN](value)
    return int(2.048 * value)

def p_gain_2bytes(value, modelclass=None, mode=None):
    limits.CHECK[pt.P_GAIN](value)
    return int(8.0 * value)

CONV[pt.D_GAIN] = (d_gain_2bytes, bytes2_d_gain)
CONV[pt.I_GAIN] = (i_gain_2bytes, bytes2_i_gain)
CONV[pt.P_GAIN] = (p_gain_2bytes, bytes2_p_gain)

def bytes2_gains(gains, modelclass=None, mode=None):
    """
        Return real values of PID gains according to
        http://support.robotis.com/en/images/product/dynamixel/mx_series/

        Kp = P_Gains / 8
        Ki = I_Gains * 1000 / 2048
        Kd = D_Gains * 4 / 1000

        """
    if not len(gains) == 3 :
        raise ValueError('Gains should have 3 values')

    return bytes2_d_gain(gains[0]), bytes2_i_gain(gains[1]), bytes2_p_gain(gains[2])

def gains_2bytes(gains, modelclass=None, mode=None):
    if not len(gains) == 3 :
        raise ValueError('Gains should have 3 values')

    return d_gain_2bytes(gains[0]), i_gain_2bytes(gains[1]), p_gain_2bytes(gains[2])

CONV[pt.GAINS] = (gains_2bytes, bytes2_gains)

# MARK Compliance Margins

def _def_bytes2_compliance_margin(control):
    def _b2b(value, modelclass=None, mode=None):
        limits.CHECK_BYTES[control](value)
        max_pos, max_deg = limits.POSITION_RANGES[modelclass]
        return (value / max_pos) * max_deg
    return _b2b

def _def_compliance_margin_2bytes(control):
    def _b2b(value, modelclass=None, mode=None):
        limits.CHECK[control](value)
        max_pos, max_deg = limits.POSITION_RANGES[modelclass]
        return int(round((value / max_deg) * max_pos))
    return _b2b

cw_compliance_margin_2bytes = _def_compliance_margin_2bytes(pt.CW_COMPLIANCE_MARGIN)
bytes2_cw_compliance_margin = _def_bytes2_compliance_margin(pt.CW_COMPLIANCE_MARGIN)
CONV[pt.CW_COMPLIANCE_MARGIN] = (cw_compliance_margin_2bytes, bytes2_cw_compliance_margin)

ccw_compliance_margin_2bytes = _def_compliance_margin_2bytes(pt.CCW_COMPLIANCE_MARGIN)
bytes2_ccw_compliance_margin = _def_bytes2_compliance_margin(pt.CCW_COMPLIANCE_MARGIN)
CONV[pt.CCW_COMPLIANCE_MARGIN] = (ccw_compliance_margin_2bytes, bytes2_ccw_compliance_margin)


# MARK Compliance Slopes

def _def_bytes2_compliance_slope(control):
    def _b2b(value, modelclass=None, mode=None):
        limits.CHECK_BYTES[control](value)
        for i in xrange(7, -1, -1):
            if (value >> i) == 1:
                return i
        return 0
    return _b2b

def _def_compliance_slope_2bytes(control):
    def _b2b(value, modelclass=None, mode=None):
        limits.CHECK[control](value)
        return 2**int(value)
    return _b2b

cw_compliance_scope_2bytes  = _def_compliance_slope_2bytes(pt.CW_COMPLIANCE_SLOPE)
bytes2_cw_compliance_scope  = _def_bytes2_compliance_slope(pt.CW_COMPLIANCE_SLOPE)

ccw_compliance_scope_2bytes = _def_compliance_slope_2bytes(pt.CCW_COMPLIANCE_SLOPE)
bytes2_ccw_compliance_scope = _def_bytes2_compliance_slope(pt.CCW_COMPLIANCE_SLOPE)

CONV[pt.CW_COMPLIANCE_SLOPE]  = ( cw_compliance_scope_2bytes, bytes2_cw_compliance_scope)
CONV[pt.CCW_COMPLIANCE_SLOPE] = (ccw_compliance_scope_2bytes, bytes2_ccw_compliance_scope)

# MARK Alarm conversions

bytes2_alarm_led = alarms.bytes2_alarm_names
alarm_led_2bytes = alarms.alarm_names_2bytes
CONV[pt.ALARM_LED] = (alarm_led_2bytes, bytes2_alarm_led)

bytes2_alarm_shutdown = alarms.bytes2_alarm_names
alarm_shutdown_2bytes = alarms.alarm_names_2bytes
CONV[pt.ALARM_SHUTDOWN] = (alarm_shutdown_2bytes, bytes2_alarm_shutdown)


# MARK Goal Acceleration

def goal_acceleration_2bytes(value, modelclass=None, mode=None):
    """in degree/s**2"""
    limits.checkbounds('goal acceleration', 0, 2180.082, value)
    return int(value/8.583)

def bytes2_goal_acceleration(value, modelclass=None, mode=None):
    """in degree/s**2"""
    limits.CHECK[pt.GOAL_ACCELERATION](value)
    return value*8.583

CONV[pt.GOAL_ACCELERATION] = (goal_acceleration_2bytes, bytes2_goal_acceleration)


# MARK Goal Torque, Current

def _def_current_2bytes(control):
    def _b2b(value, modelclass=None, mode=None):
        """in A"""
        limits.CHECK[control](value)
        return int(value/0.0045 + 2048)
    return _b2b

def _def_bytes2_current(control):
    def _b2b(value, modelclass=None, mode=None):
        """in A"""
        limits.CHECK_BYTES[control](value)
        return 0.0045 * (value - 2048)
    return _b2b

goal_torque_2bytes = _def_current_2bytes(pt.GOAL_TORQUE)
bytes2_goal_torque = _def_bytes2_current(pt.GOAL_TORQUE)
CONV[pt.GOAL_TORQUE] = (goal_torque_2bytes, bytes2_goal_torque)

current_2bytes = _def_current_2bytes(pt.CURRENT)
bytes2_current = _def_bytes2_current(pt.CURRENT)
CONV[pt.CURRENT] = (current_2bytes, bytes2_current)


def sensed_current_2bytes(value, modelclass=None, mode=None):
    """in A - should not be useful (since you can't write sensed current)"""
    limits.CHECK[pt.SENSED_CURRENT](value)
    return int(value/0.01 + 512)

def bytes2_sensed_current(value, modelclass=None, mode=None):
    """in A"""
    limits.CHECK_BYTES[pt.SENSED_CURRENT](value)
    return 0.01 * (value - 512)


# Mark Boolean

def _def_bool_2bytes(control):
    def _b2b(value, modelclass=None, mode=None):
        limits.CHECK_BYTES[control](value)
        return int(value)
    return _b2b

def _def_bytes2_bool(control):
    def _b2b(value, modelclass=None, mode=None):
        limits.CHECK[control](value)
        return bool(value)
    return _b2b

bytes2_registered = _def_bytes2_bool(pt.REGISTERED)
registered_2bytes = _def_bool_2bytes(pt.REGISTERED)
CONV[pt.REGISTERED] = (registered_2bytes, bytes2_registered)

bytes2_lock = _def_bytes2_bool(pt.LOCK)
lock_2bytes = _def_bool_2bytes(pt.LOCK)
CONV[pt.LOCK] = (lock_2bytes, bytes2_lock)

bytes2_led = _def_bytes2_bool(pt.LED)
led_2bytes = _def_bool_2bytes(pt.LED)
CONV[pt.LED] = (led_2bytes, bytes2_led)

bytes2_moving = _def_bytes2_bool(pt.MOVING)
moving_2bytes = _def_bool_2bytes(pt.MOVING)
CONV[pt.MOVING] = (moving_2bytes, bytes2_moving)

bytes2_torque_control_mode_enable = _def_bytes2_bool(pt.TORQUE_CONTROL_MODE_ENABLE)
torque_control_mode_enable_2bytes = _def_bool_2bytes(pt.TORQUE_CONTROL_MODE_ENABLE)
CONV[pt.TORQUE_CONTROL_MODE_ENABLE] = (torque_control_mode_enable_2bytes, bytes2_torque_control_mode_enable)

bytes2_torque_enable = _def_bytes2_bool(pt.TORQUE_ENABLE)
torque_enable_2bytes = _def_bool_2bytes(pt.TORQUE_ENABLE)
CONV[pt.TORQUE_ENABLE] = (torque_enable_2bytes, bytes2_torque_enable)
