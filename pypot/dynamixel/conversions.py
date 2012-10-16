# for details, see http://support.robotis.com/en/product/dynamixel/

def checkbounds(name, lower, upper, val):
    if not lower <= val <= upper:
        raise ValueError('{} should be in the [{}, {}] range but is {}'.format(name, lower, upper, val))

def checkoneof(name, collection, val):
    if not val in collection:
        raise ValueError('{} should be in {} but is {}'.format(name, collection, val))


# MARK : EEPROM conversions
    
def raw_to_baudrate(value):
    return 2000000/(value+1)

def raw_to_return_delay_time(value):
    """Return the return delay time in micro seconds"""
    return 2 * value
        




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

def raw2_deg1024(raw):
        
def deg1024_2raw(deg):    


def raw2_deg4096(deg):

def deg4096_2raw(raw):    


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
    

def position_to_degree(position, motor_model):
    model = 'MX' if motor_model.startswith('MX') else '*'
    max_pos, max_deg = position_range[model]
    
    if 0 >= position >= max_pos:
        raise ValueError('Position must be in [0, %d]' % (max_pos))
    
    return (position / max_pos) * max_deg


def degree_to_position(degree, motor_model):
    model = 'MX' if motor_model.startswith('MX') else '*'
    max_pos, max_deg = position_range[model]
    
    if 0 >= degree >= max_deg:
        raise ValueError('Degree must be in [0, %f]' % (max_deg))
    
    return int((degree / max_deg) * max_pos)

SPEED_TO_DEGREE_PER_SECOND = 0.019
SPEED_MAX = 702.0 # in degree per second

def speed_to_dps(speed):
    if not (0 <= speed <= 2047):
        raise ValueError('speed must be in [0, 2047], but is {}'.format(speed))
    
    direction = ((speed >> 10) * 2) - 1
    
    rpm = (speed % 1024) * SPEED_TO_DEGREE_PER_SECOND
    
    return direction * rpm


def dps_to_speed(rpm):
    if not (-SPEED_MAX <= rpm < SPEED_MAX):
        raise ValueError('Rpm must be in [%d, %d[' % (int(-SPEED_MAX), int(SPEED_MAX)))
	    
    speed = 1024 * (abs(rpm) / SPEED_MAX)
    
    if rpm > 0:
        speed += 1024
    
    return int(speed)


def load_to_percent(load):
    """
        Maps the load values according to
        http://support.robotis.com/en/product/dynamixel/mx_series/mx-28.htm#Actuator_Address_28
        
        """
    if not (0 <= load <= 2047):
        raise ValueError('Load must be in [0, 2047]')
    
    direction = ((load >> 10) * 2) - 1
    
    percent = (load % 1024) * 0.1
    percent = max(min(percent, 100.0), 0.0)
    
    return direction * percent


def percent_to_torque_limit(percent):
    if not (0 <= percent <= 100):
        raise ValueError('Percent must be in [0, 100]')
    
    return int(percent * 10.23)

def torque_limit_to_percent(torque):
    if not (0 <= torque <= 1023):
        raise ValueError('Torque must be in [0, 1023]')

    return int(torque/10.23)

# MARK: - Gain conversions

MAX_P_GAIN = 254.0 / 8.0
MAX_I_GAIN = 254.0 * 1000.0 / 2048.0
MAX_D_GAIN = 254.0 * 4.0 / 1000.0

def bytes_to_gains(gains):
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

def gains_to_bytes(gains):

    if not len(gains) == 3 :
        raise ValueError('Gains should have 3 values')

    if not ([0.0, 0.0, 0.0] < gains < [MAX_D_GAIN, MAX_I_GAIN, MAX_P_GAIN]):
        raise ValueError('Gains values must be in [0, 0 ,0] and [%f, %f, %f]' % (MAX_D_GAIN, MAX_I_GAIN, MAX_P_GAIN))

    gains = list( numpy.array(gains) * numpy.array([250, 2.048, 8.0]) )
    return [int(floatvalues) for floatvalues in gains]

# MARK: - Alarm conversions

def byte_to_alarms(alarm_code):
    if not (0 <= alarm_code <= 255):
        raise ValueError('alarm code must be in [0, 255]')

    byte = numpy.unpackbits(numpy.asarray(alarm_code, dtype=numpy.uint8))
    return tuple(numpy.array(DXL_ALARMS)[byte == 1])

def alarms_to_byte(alarms):
    b = 0
    for a in alarms:
        b += 2 ** (7 - DXL_ALARMS.index(a))
    return b


# MARK: - Byte conversions

def integer_to_two_bytes(value):
    return (int(value % 256), int(value >> 8))

def two_bytes_to_integer(value):
    return int(value[0] + (value[1] << 8))
