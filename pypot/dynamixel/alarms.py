# MARK: - Alarm conversions

DXL_ALARMS = [
    "Input Voltage Error",
    "Angle Limit Error",
    "Overheating Error",
    "Range Error",
    "Checksum Error",
    "Overload Error",
    "Instruction Error",
]

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