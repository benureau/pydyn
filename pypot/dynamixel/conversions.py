import numpy

import limits
import protocol

# for details, see http://support.robotis.com/en/product/dynamixel/




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

def baudrate_2raw(value, modelclass):
    return baudrate_fun[modelclass][0](value)

def raw2_baudrate(value, modelclass):
    return baudrate_fun[modelclass][1](value)


# MARK Return delay time

def return_delay_time_2raw(value):
    """in microseconds"""
    limits.checkbounds('return delay time', 0, 508, value)
    return int(value/2)

def raw2_return_delay_time(value):
    """in microseconds"""
    limits.checkbounds('return delay time raw', 0, 254, value)
    return 2*value


# MARK Voltage

def voltage_2raw(value):
    """Return the voltage in volt"""
    limits.checkbounds('voltage', 0, 25.5, value)
    return int(10.0*value)

def raw2_voltage(value):
    """Return the voltage in volt"""
    limits.checkbounds('voltage raw', 0, 255, value)
    return value/10.0


# MARK Torque

def raw2_torque(value):
    """Return the voltage in volt"""
    limits.checkbounds('torque', 0, 1023, value)
    return 100*value/1023.0

def torque_2raw(value):
    """Return the voltage in volt"""
    limits.checkbounds('torque raw', 0, 100, value)
    return int(value/100*1023)


# MARK : RAM conversions

# MARK Position

def raw2_deg(raw, modelclass):
    max_pos, max_deg = limits.position_range[modelclass]
    limits.checkbounds('position raw', 0, max_pos, raw)
    return (raw / max_pos) * max_deg

def deg_2raw(deg, modelclass):
    max_pos, max_deg = limits.position_range[modelclass]
    limits.checkbounds('position', 0, max_deg, deg)
    return int((deg / max_deg) * max_pos)

# MARK Speed

speedratio = {
    'AX': 6*0.111,
    'RX': 6*0.111,
    'EX': 6*0.111,
    'MX': 6*0.11445,
}

def raw2_positivedps(raw, modelclass):
    """
        Raw to degree per second for speed
        raw values are in [0, 1023], and 1023 ~ 117.07 rpm (MX) or 114 rpm
        (AX and RX)
        """
    limits.checkbounds('positive speed raw', 0, 1023, raw)
    return raw*speedratio[modelclass]

def movingdps_2raw(dps, modelclass):
    """
        Degree per second for speed to raw value
        raw values are in [0, 1023], and 1023 ~ 117.07 rpm (MX) or 114 rpm
        (AX and RX)
        """
    max_speed = 1023*speedratio[modelclass]
    limits.checkbounds('positive speed', 0, max_speed, dps)
    return int(dps/speedratio[modelclass])

def raw2_cwccwdps(raw, modelclass):
    """
        Raw to degree per second for CW/CCW speed

        Robotis manual :
            If a value is in the rage of 0~1023 then the motor rotates to the CCW direction.
            If a value is in the rage of 1024~2047 then the motor rotates to the CW direction.
            The 10th bit becomes the direction bit to control the direction; 0 and 1024 are equal.

        a unit equals (about) 0.11445 rpm = 0.6867 dps (MX) and 0.111 rpm = 0.666 dps (AX and RX)

        """
    limits.checkbounds('cw/ccw speed raw', 0, 2047, raw)
    direction = ((raw >> 10) * 2) - 1
    speed = raw2_positivedps(raw % 1024, modelclass)

    return direction * speed

def cwccwdps_2raw(dps, modelclass):
    """
        degree per second for CW/CCW speed to raw value

        Robotis manual :
            If a value is in the rage of 0~1023 then the motor rotates to the CCW direction.
            If a value is in the rage of 1024~2047 then the motor rotates to the CW direction.
            The 10th bit becomes the direction bit to control the direction; 0 and 1024 are equal.

        a unit equals (about) 0.11445 rpm = 0.6867 dps (MX) and 0.111 rpm = 0.666 dps (AX and RX)

        """
    max_speed = 1023*speedratio[modelclass]
    limits.checkbounds('cw/ccw speed', -max_speed, max_speed, dps)

    if dps > 0:
        return int(1024 + abs(dps)/speedratio[modelclass])
    else:
        return int(abs(dps)/speedratio[modelclass])

# MARK Load

def raw2_load(value):
    """return the load into signed torque percent"""
    limits.checkbounds('load raw', 0, 2047, value)
    direction = ((value >> 10) * 2) - 1

    return direction * raw2_torque(value % 1024)

def load_2raw(value):
    limits.checkbounds('load', -100, 100, value)

    if value > 0:
        return int(1024 + 1023*abs(value)/100.0)
    else:
        return int(1023*abs(value)/100.0)


# MARK: - Gain conversions

MAX_P_GAIN = 254.0 / 8.0
MAX_I_GAIN = 254.0 * 1000.0 / 2048.0
MAX_D_GAIN = 254.0 * 4.0 / 1000.0

def raw2_dgain(raw):
    limits.checkbounds('p_gain raw', 0, 254, raw)
    return 0.004 * raw

def raw2_igain(raw):
    limits.checkbounds('i_gain raw', 0, 254, raw)
    return raw / 2.048

def raw2_pgain(raw):
    limits.checkbounds('d_gain raw', 0, 254, raw)
    return 0.125 * raw

def dgain_2raw(value):
    limits.checkbounds('p_gain', 0, MAX_P_GAIN, value)
    return int(250.0 * value)

def igain_2raw(value):
    limits.checkbounds('i_gain', 0, MAX_I_GAIN, value)
    return int(2.048 * value)

def pgain_2raw(value):
    limits.checkbounds('d_gain', 0, MAX_D_GAIN, value)
    return int(8.0 * value)


def raw2_gains(gains):
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

def gains_2raw(gains):

    if not len(gains) == 3 :
        raise ValueError('Gains should have 3 values')

    return dgain_2raw(gains[0]), igain_2raw(gains[1]), pgain_2raw(gains[2])


# MARK: - Alarm conversions

def raw2_alarm_codes(value):
    """This unpack a single integer into a list of error code"""
    limits.checkbounds('alarm code raw', 0, 127, value)

    return numpy.unpackbits(numpy.asarray(value, dtype=numpy.uint8))

def raw2_alarm_names(value):
    """This unpack a single integer into a list of error names"""
    byte = raw2_alarm_codes(value)
    return tuple(numpy.array(protocol.DXL_ALARMS)[byte == 1])

def alarm_names_2raw(value):
    b = 0
    for a in value:
        b += 2 ** (7 - protocol.DXL_ALARMS.index(a))
    return b

def alarm_codes_2raw(value):
    b = 0
    for c in value:
        b += 2 ** c
    return b

def alarm_code_2name(value):
    """value is a integer representing a single alarmcode"""
    return protocol.DXL_ALARMS[value]

def name2_alarm_code(value):
    """value is a integer representing a single alarmcode"""
    return protocol.DXL_ALARMS.index(value)


# MARK Current

def current_2raw(value):
    """in A"""
    limits.checkbounds('current', -9.2115, 9.2115, value)
    return int(value/0.0045 + 2048)
    
def raw2_current(value):
    """in A"""
    limits.checkbounds('current raw', 0, 4095, value)
    return 0.0045 * (value - 2048)

def sensed_current_2raw(value):
    """in A - should not be useful (since you can't write sensed current)"""
    limits.checkbounds('sensed current', -5.12, 5.11, value)
    return int(value/0.01 + 512)

def raw2_sensed_current(value):
    """in A"""
    limits.checkbounds('sensed current raw', 0, 1023, value)
    return 0.01 * (value - 512)
