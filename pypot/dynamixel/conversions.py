import limits
import protocol
import alarms

# for details, see http://support.robotis.com/en/product/dynamixel/

# MARK Dynamic

def raw2_value(name, value, mmem = None):
    return globals()['raw2_' + name.lower()](int(value), mmem)
    
def value_2raw(name, value, mmem = None):
    return globals()[name.lower()+'_2raw'](value, mmem)


# MARK Model Number

def raw2_model_number(value, mmem = None):
    limits.checkoneof('model number', [12, 18, 44, 10, 24, 28, 64, 29, 54, 320, 107], value)
    return int(value)

def model_number_2raw(value, mmem = None):
    limits.checkoneof('model number', [12, 18, 44, 10, 24, 28, 64, 29, 54, 320, 107], value)
    return int(value)


# MARK Firmware

def raw2_version(value, mmem = None):
    limits.checkbounds('version', 0, 253, value)
    return int(value)

def version_2raw(value, mmem = None):
    limits.checkbounds('version', 0, 253, value)
    return int(value)


# MARK ID

def raw2_id(value, mmem = None):
    limits.checkbounds('id', 0, 253, value)
    return int(value)

def id_2raw(value, mmem = None):
    limits.checkbounds('id', 0, 253, value)
    return int(value)


# MARK Status Return Level

def raw2_status_return_level(value, mmem = None):
    limits.checkoneof('status return level', [0, 1, 2], value)
    return int(value)

def status_return_level_2raw(value, mmem = None):
    limits.checkoneof('status return level', [0, 1, 2], value)
    return int(value)


# MARK Baudrate

def raw2_baudrate_axrx(value):
    limits.checkbounds('baudrate raw', 0, 254, value)
    return 2000000.0/(value + 1)

def baudrate_axrx_2raw(value):
    return int(2000000.0/value - 1)

baudrate_mx = {
    250 : 2250000,
    251 : 2500000,
    252 : 3000000,
}

mx_baudrate = {
    2250000 : 250,
    2500000 : 251,
    3000000 : 252,
}

def raw2_baudrate_mx(value):
    limits.checkbounds('baudrate raw', 0, 254, value)
    try:
        return baudrate_mx[value]
    except KeyError:
        return 2000000.0/(value + 1)

def baudrate_mx_2raw(value):
    try:
        return max_baudrate[int(value)]
    except KeyError:
        return int(2000000.0/value - 1)

baudrate_fun = {
    'AX' : (raw2_baudrate_axrx, baudrate_axrx_2raw),
    'RX' : (raw2_baudrate_axrx, baudrate_axrx_2raw),
    'EX' : (raw2_baudrate_axrx, baudrate_axrx_2raw),
    'MX' : (raw2_baudrate_mx,   baudrate_mx_2raw),
}

def baud_rate_2raw(value, mmem):
    return baudrate_fun[mmem.modelclass][1](value)

def raw2_baud_rate(value, mmem):
    return baudrate_fun[mmem.modelclass][0](value)


# MARK Return delay time

def return_delay_time_2raw(value, mmem = None):
    """in microseconds"""
    limits.checkbounds('return delay time', 0, 508, value)
    return int(value/2)

def raw2_return_delay_time(value, mmem = None):
    """in microseconds"""
    limits.checkbounds('return delay time raw', 0, 254, value)
    return 2*value


# MARK Voltage

def voltage_2raw(value, mmem = None):
    """Return the voltage in volt"""
    limits.checkbounds('voltage', 0, 25.5, value)
    return int(10.0*value)

def raw2_voltage(value, mmem = None):
    """Return the voltage in volt"""
    limits.checkbounds('voltage raw', 0, 255, value)
    return value/10.0

highest_limit_voltage_2raw = voltage_2raw
raw2_highest_limit_voltage = raw2_voltage

lowest_limit_voltage_2raw = voltage_2raw
raw2_lowest_limit_voltage = raw2_voltage

present_voltage_2raw = voltage_2raw
raw2_present_voltage = raw2_voltage


# MARK Torque

def raw2_torque(value, mmem = None):
    """Return the voltage in volt"""
    limits.checkbounds('torque', 0, 1023, value)
    return 100*value/1023.0

def torque_2raw(value, mmem = None):
    """Return the voltage in volt"""
    limits.checkbounds('torque raw', 0, 100, value)
    return int(value/100*1023)

max_torque_2raw = torque_2raw
raw2_max_torque = raw2_torque

torque_limit_2raw = torque_2raw
raw2_torque_limit = raw2_torque

punch_2raw = torque_2raw
raw2_punch = raw2_torque


# MARK Position

def raw2_deg(value, mmem):
    max_pos, max_deg = limits.position_range[mmem.modelclass]
    limits.checkbounds('position raw', 0, max_pos, value)
    return (value / max_pos) * max_deg

def deg_2raw(value, mmem):
    max_pos, max_deg = limits.position_range[mmem.modelclass]
    limits.checkbounds('position', 0, max_deg, value)
    return int((value / max_deg) * max_pos)

def safedeg_2raw(value, mmem):
    min_angle = raw2_deg(mmem[protocol.DXL_CW_ANGLE_LIMIT], mmem)
    max_angle = raw2_deg(mmem[protocol.DXL_CCW_ANGLE_LIMIT], mmem)
    limits.checkbounds('safe position', min_angle, max_angle, value)
    return deg_2raw(value, mmem)


raw2_position = raw2_deg
position_2raw = safedeg_2raw

raw2_goal_position = raw2_deg
goal_position_2raw = safedeg_2raw

raw2_present_position = raw2_deg
present_position_2raw = deg_2raw # useless

raw2_cw_angle_limit   = raw2_deg
cw_angle_limit_2raw   = deg_2raw

raw2_ccw_angle_limit  = raw2_deg
ccw_angle_limit_2raw  = deg_2raw


# MARK Speed
        
speedratio = {
    'AX': 6*0.111,
    'RX': 6*0.111,
    'EX': 6*0.111,
    'MX': 6*0.11445,
}

def raw2_movingdps(value, mmem):
    """
        Raw to degree per second for speed
        raw values are in [0, 1023], and 1023 ~ 117.07 rpm (MX) or 114 rpm
        (AX and RX)
        """
    limits.checkbounds('positive speed raw', 0, 1023, value)
    return value*speedratio[mmem.modelclass]

def movingdps_2raw(value, mmem):
    """
        Degree per second for speed to raw value
        raw values are in [0, 1023], and 1023 ~ 117.07 rpm (MX) or 114 rpm
        (AX and RX)
        """
    max_speed = 1023*speedratio[mmem.modelclass]
    limits.checkbounds('positive speed', 0, max_speed, value)
    return int(value/speedratio[mmem.modelclass])

def raw2_moving_speed(value, mmem):
    if mmem.mode == 'wheel' and mmem.modelclass in ('RX', 'AX'):
        return raw2_present_load(value)
    else:
        return raw2_movingdps(value, mmem)

def moving_speed_2raw(value, mmem):
    if mmem.mode == 'wheel' and mmem.modelclass in ('RX', 'AX'):
        return present_load_2raw(value)
    else:
        return movingdps_2raw(value, mmem)

def raw2_cwccwdps(value, mmem):
    """
        Raw to degree per second for CW/CCW speed

        Robotis manual :
            If a value is in the rage of 0~1023 then the motor rotates to the CCW direction.
            If a value is in the rage of 1024~2047 then the motor rotates to the CW direction.
            The 10th bit becomes the direction bit to control the direction; 0 and 1024 are equal.

        a unit equals (about) 0.11445 rpm = 0.6867 dps (MX) and 0.111 rpm = 0.666 dps (AX and RX)

        """
    limits.checkbounds('cw/ccw speed raw', 0, 2047, value)
    direction = ((value >> 10) * 2) - 1
    speed = raw2_moving_speed(value % 1024, mmem)

    return direction * speed

def cwccwdps_2raw(value, mmem):
    """
        degree per second for CW/CCW speed to raw value

        Robotis manual :
            If a value is in the rage of 0~1023 then the motor rotates to the CCW direction.
            If a value is in the rage of 1024~2047 then the motor rotates to the CW direction.
            The 10th bit becomes the direction bit to control the direction; 0 and 1024 are equal.

        a unit equals (about) 0.11445 rpm = 0.6867 dps (MX) and 0.111 rpm = 0.666 dps (AX and RX)

        """
    max_speed = 1023*speedratio[mmem.modelclass]
    limits.checkbounds('cw/ccw speed', -max_speed, max_speed, value)

    if value > 0:
        return int(1024 + abs(value)/speedratio[mmem.modelclass])
    else:
        return int(abs(value)/speedratio[mmem.modelclass])

present_speed_2raw = cwccwdps_2raw
raw2_present_speed = raw2_cwccwdps


# MARK Load

def raw2_present_load(value, mmem = None):
    """return the load into signed torque percent"""
    limits.checkbounds('load raw', 0, 2047, value)
    direction = ((value >> 10) * 2) - 1

    return direction * raw2_torque(value % 1024)

def present_load_2raw(value, mmem = None):
    limits.checkbounds('load', -100, 100, value)

    if value > 0:
        return int(1024 + 1023*abs(value)/100.0)
    else:
        return int(1023*abs(value)/100.0)


# MARK: - Gain conversions

MAX_P_GAIN = 254.0 / 8.0
MAX_I_GAIN = 254.0 * 1000.0 / 2048.0
MAX_D_GAIN = 254.0 * 4.0 / 1000.0

def raw2_d_gain(raw, mmem = None):
    limits.checkbounds('p_gain raw', 0, 254, raw)
    return 0.004 * raw

def raw2_i_gain(raw, mmem = None):
    limits.checkbounds('i_gain raw', 0, 254, raw)
    return raw / 2.048

def raw2_p_gain(raw, mmem = None):
    limits.checkbounds('d_gain raw', 0, 254, raw)
    return 0.125 * raw

def d_gain_2raw(value, mmem = None):
    limits.checkbounds('p_gain', 0, MAX_P_GAIN, value)
    return int(250.0 * value)

def i_gain_2raw(value, mmem = None):
    limits.checkbounds('i_gain', 0, MAX_I_GAIN, value)
    return int(2.048 * value)

def p_gain_2raw(value, mmem = None):
    limits.checkbounds('d_gain', 0, MAX_D_GAIN, value)
    return int(8.0 * value)

def raw2_gains(gains, mmem = None):
    """
        Return real values of PID gains according to
        http://support.robotis.com/en/images/product/dynamixel/mx_series/

        Kp = P_Gains / 8
        Ki = I_Gains * 1000 / 2048
        Kd = D_Gains * 4 / 1000

        """
    if not len(gains) == 3 :
        raise ValueError('Gains should have 3 values')

    return raw2_dgain(gains[0]), raw2_dgain(gains[1]), raw2_dgain(gains[2])

def gains_2raw(gains, mmem = None):

    if not len(gains) == 3 :
        raise ValueError('Gains should have 3 values')

    return dgain_2raw(gains[0]), igain_2raw(gains[1]), pgain_2raw(gains[2])


# MARK Compliance Margins

def compliance_margin_2raw(value, mmem = None):
    limits.checkbounds('compliance margin', 0, raw2_deg(255, mmem), value)
    return raw2_deg(value, mmem)

def raw2_compliance_margin(value, mmem = None):
    limits.checkbounds('compliance margin raw', 0, 255, value)
    return deg_2raw(value, mmem)


# MARK Compliance Slopes

def raw2_compliance_slope(value, mmem = None):
    limits.checkbounds('compliance slope raw', 0, 254, value)
    for i in xrange(7, -1, -1):
        if (value >> i) == 1:
            return i
    return 0

def compliance_slope_2raw(value, mmem = None):
    limits.checkbounds('compliance slope', 0, 7, value)
    return 2**int(value)


# MARK Alarm conversions

from alarms import raw2_alarm_codes, raw2_alarm_names, alarm_codes_2raw, alarm_names_2raw

raw2_alarm_led = raw2_alarm_names
alarm_led_2raw = alarm_names_2raw

raw2_alarm_shutdown = raw2_alarm_names
alarm_shutdown_2raw = alarm_names_2raw


# MARK Temperature

def present_temperature_2raw(value, mmem = None):
    return int(value)

def raw2_present_temperature(value, mmem = None):
    return value

def highest_limit_temperature_2raw(value, mmem = None):
    limits.checkbounds('present_temperature', 10, 99, value)
    return int(value)

def raw2_highest_limit_temperature(value, mmem = None):
    limits.checkbounds('present_temperature', 10, 99, value)
    return value


# MARK Goal Torque

def goal_torque_2raw(value, mmem = None):
    """in A"""
    limits.checkbounds('goal torque', -4.6035, 4.6035, value)
    return int(value/0.0045 + 1024)

def raw2_goal_torque(value, mmem = None):
    """in A"""
    # it says that goal torque can't be bigger than torque limit
    # but since each have different units, it's difficult
    # to test.
    limits.checkbounds('goal torque raw', 0, 2047, value)
    return 0.0045 * (value - 1024)


# MARK Goal Acceleration

def goal_acceleration_2raw(value, mmem = None):
    """in degree/s**2"""
    limits.checkbounds('goal acceleration', 0, 2180.082, value)
    return int(value*8.583)

def raw2_goal_acceleration(value, mmem = None):
    """in degree/s**2"""
    limits.checkbounds('goal acceleration raw', 0, 254, value)
    return value/8.583


# MARK Current

def current_2raw(value, mmem = None):
    """in A"""
    limits.checkbounds('current', -9.216, 9.2115, value)
    return int(value/0.0045 + 2048)

def raw2_current(value, mmem = None):
    """in A"""
    limits.checkbounds('current raw', 0, 4095, value)
    return 0.0045 * (value - 2048)
    
def sensed_current_2raw(value, mmem = None):
    """in A - should not be useful (since you can't write sensed current)"""
    limits.checkbounds('sensed current', -5.12, 5.11, value)
    return int(value/0.01 + 512)

def raw2_sensed_current(value, mmem = None):
    """in A"""
    limits.checkbounds('sensed current raw', 0, 1023, value)
    return 0.01 * (value - 512)


# Mark Boolean

def bool_2raw(value, mmem = None):
    limits.checkoneof('boolean value raw', [0, 1], value)
    return int(value)

def raw2_bool(value, mmem = None):
    limits.checkoneof('boolean value', [False, True], value)
    return bool(value)

raw2_registered = raw2_bool
registered_2raw = bool_2raw

raw2_lock = raw2_bool
lock_2raw = bool_2raw

raw2_led = raw2_bool
led_2raw = bool_2raw

raw2_moving = raw2_bool
moving_2raw = bool_2raw

raw2_torque_control_mode_enable = raw2_bool
torque_control_mode_enable_2raw = bool_2raw

raw2_torque_enable = raw2_bool
torque_enable_2raw = bool_2raw

