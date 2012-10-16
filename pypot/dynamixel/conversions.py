import limits

# for details, see http://support.robotis.com/en/product/dynamixel/

def checkbounds(name, lower, upper, val):
    if not lower <= val <= upper:
        raise ValueError('{} should be in the [{}, {}] range but is {}'.format(name, lower, upper, val))

def checkbounds_mode(name, lower, upper, val, mode):
    if not lower <= val <= upper:
        raise ValueError('in {} mode, {} should be in the [{}, {}] range but is {}'.format(mode, name, lower, upper, val))

def checkoneof(name, collection, val):
    if not val in collection:
        raise ValueError('{} should be in {} but is {}'.format(name, collection, val))


# MARK Baudrate

def raw2_baudrate_axrx(value):
    checkbounds('baudrate', 0, 254, value)
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
    checkbounds('baudrate', 0, 254, value)
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
    'MX' : (raw2_baudrate_mx,   baudrate_mx_2raw),
}

def baudrate_2raw(value, modelclass):
    return baudrate_fun[modelclass][0](value)

def raw2_baudrate(value, modelclass):
    return baudrate_fun[modelclass][1](value)


# MARK Return delay time

def return_delay_time_2raw(value):
    """in microseconds"""
    checkbounds('return delay time', 0, 508, value)
    return int(value/2)

def raw2_return_delay_time(value):
    """in microseconds"""
    checkbounds('return delay time', 0, 254, value)
    return 2*value


# MARK Voltage

def voltage_2raw(value):
    """Return the voltage in volt"""
    checkbounds('voltage', 0, 25.5, value)
    return int(10.0*value)

def raw2_voltage(value):
    """Return the voltage in volt"""
    checkbounds('voltage', 0, 255, value)
    return value/10.0


# MARK Torque

def raw2_torque(value):
    """Return the voltage in volt"""
    checkbounds('torque', 0, 1023, value)
    return 100*value/1023.0

def torque_2raw(value):
    """Return the voltage in volt"""
    checkbounds('torque', 0, 100, value)
    return int(value/100*1023)


# MARK : RAM conversions

# MARK Position

def raw2_deg(raw, modelclass):
    max_pos, max_deg = limits.position_range[modelclass]
    return (position / max_pos) * max_deg

def deg_2raw(deg, modelclass):
    max_pos, max_deg = limits.position_range[modelclass]
    return int((position / max_deg) * max_pos)

# MARK Speed

speedratio = {
    'AX': 0.111,
    'RX': 0.111,
    'MX': 0.11445,
}

def raw2_positivedps(raw, modelclass):
    """
        Raw to degree per second for speed
        raw values are in [0, 1023], and 1023 ~ 117.07 rpm (MX) or 114 rpm
        (AX and RX)
        """
    checkbounds('positive speed', 0, 1023, raw)
    return raw*6*speedratio[modelclass]

def raw2_dps(raw):
    """
        Raw to degree per second for CW/CCW speed

        Robotis manual :
            If a value is in the rage of 0~1023 then the motor rotates to the CCW direction.
            If a value is in the rage of 1024~2047 then the motor rotates to the CW direction.
            The 10th bit becomes the direction bit to control the direction; 0 and 1024 are equal.

        a unit equals (about) 0.11445 rpm = 0.6867 dps (MX) and 0.111 rpm = 0.666 dps (AX and RX)

        """
    checkbounds('cw/ccw speed', 0, 2047, raw)
    direction = ((speed >> 10) * 2) - 1
    speed = raw2_positivedps(raw)

    return direction * speed


# MARK Load

def raw2_load(value):
    """return the load into signed torque percent"""
    checkbounds('load', 0, 2047, value)
    direction = ((value >> 10) * 2) - 1

    return direction * raw2_torque(value % 1024)

def load_2raw(value):
    checkbounds('load', -100, 100, value)


    if value > 0:
        return int(1024 + 1023*abs(value)/100.0)
    else:
        return int(1023*abs(value)/100.0)


# MARK: - Gain conversions

MAX_P_GAIN = 254.0 / 8.0
MAX_I_GAIN = 254.0 * 1000.0 / 2048.0
MAX_D_GAIN = 254.0 * 4.0 / 1000.0

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
    if ((min(gains) < 0) or (max(gains) > 254)):
        raise ValueError('Gains values must be in [0,254]')

    return list( numpy.array(gains) * numpy.array([0.004, 1000.0 / 2048, 0.125]) )

def gains_2raw(gains):

    if not len(gains) == 3 :
        raise ValueError('Gains should have 3 values')

    if not ([0.0, 0.0, 0.0] < gains < [MAX_D_GAIN, MAX_I_GAIN, MAX_P_GAIN]):
        raise ValueError('Gains values must be in [0, 0 ,0] and [%f, %f, %f]' % (MAX_D_GAIN, MAX_I_GAIN, MAX_P_GAIN))

    gains = list( numpy.array(gains) * numpy.array([250, 2.048, 8.0]) )
    return [int(floatvalues) for floatvalues in gains]


# MARK: - Alarm conversions

def raw2_alarms(alarm_code):
    if not (0 <= alarm_code <= 255):
        raise ValueError('alarm code must be in [0, 255]')

    byte = numpy.unpackbits(numpy.asarray(alarm_code, dtype=numpy.uint8))
    return tuple(numpy.array(DXL_ALARMS)[byte == 1])

def alarms_2raw(alarms):
    b = 0
    for a in alarms:
        b += 2 ** (7 - DXL_ALARMS.index(a))
    return b


# MARK: - Byte conversions

def integer_to_two_bytes(value):
    return (int(value % 256), int(value >> 8))

def two_bytes_to_integer(value):
    return int(value[0] + (value[1] << 8))
