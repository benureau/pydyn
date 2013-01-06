# For details on the dynamixel protocol you should refer to
# http://support.robotis.com/en/product/dxl_main.htm

DXL_INSTRUCTIONS = {
    'PING'      : 0x01,
    'READ_DATA' : 0x02,
    'WRITE_DATA': 0x03,
    'SYNC_WRITE': 0x83,
    'SYNC_READ' : 0x84,
}

DXL_BROADCAST = 0xFE

DXL_INPUT_VOLTAGE_ERROR = 0
DXL_ANGLE_LIMIT_ERROR   = 1
DXL_OVERHEATING_ERROR   = 2
DXL_RANGE_ERROR         = 3
DXL_CHECKSUM_ERROR      = 4
DXL_OVERLOAD_ERROR      = 5
DXL_INSTRUCTION_ERROR   = 6


DXL_CONTROLS = {
    # EEPROM
    'MODEL_NUMBER':               {'address': 0x00, '# data': 1, 'size': 2, 'ram': False},
    'VERSION':                    {'address': 0x02, '# data': 1, 'size': 1, 'ram': False},
    'ID':                         {'address': 0x03, '# data': 1, 'size': 1, 'ram': False},
    'BAUD_RATE':                  {'address': 0x04, '# data': 1, 'size': 1, 'ram': False},
    'RETURN_DELAY_TIME':          {'address': 0x05, '# data': 1, 'size': 1, 'ram': False},
    'CW_ANGLE_LIMIT':             {'address': 0x06, '# data': 1, 'size': 2, 'ram': False},
    'CCW_ANGLE_LIMIT':            {'address': 0x08, '# data': 1, 'size': 2, 'ram': False},
    'ANGLE_LIMITS':               {'address': 0x06, '# data': 2, 'size': 2, 'ram': False, 'components': ['CW_ANGLE_LIMIT', 'CCW_ANGLE_LIMIT']},
    'DRIVE_MODE':                 {'address': 0x0A, '# data': 1, 'size': 1, 'ram': False}, # EX only
    'HIGHEST_LIMIT_TEMPERATURE':  {'address': 0x0B, '# data': 1, 'size': 1, 'ram': False},
    'LOWEST_LIMIT_VOLTAGE':       {'address': 0x0C, '# data': 1, 'size': 1, 'ram': False},
    'HIGHEST_LIMIT_VOLTAGE':      {'address': 0x0D, '# data': 1, 'size': 1, 'ram': False},
    'VOLTAGE_LIMITS':             {'address': 0x0C, '# data': 2, 'size': 1, 'ram': False, 'components': ['LOWEST_LIMIT_VOLTAGE', 'HIGHEST_LIMIT_VOLTAGE']},
    'MAX_TORQUE':                 {'address': 0x0E, '# data': 1, 'size': 2, 'ram': False},
    'STATUS_RETURN_LEVEL':        {'address': 0x10, '# data': 1, 'size': 1, 'ram': False},
    'ALARM_LED':                  {'address': 0x11, '# data': 1, 'size': 1, 'ram': False},
    'ALARM_SHUTDOWN':             {'address': 0x12, '# data': 1, 'size': 1, 'ram': False},

    # RAM
    'TORQUE_ENABLE':              {'address': 0x18, '# data': 1, 'size': 1, 'ram': True},
    'LED':                        {'address': 0x19, '# data': 1, 'size': 1, 'ram': True},

    # MX series
    'D_GAIN':                     {'address': 0x1A, '# data': 1, 'size': 1, 'ram': True},
    'I_GAIN':                     {'address': 0x1B, '# data': 1, 'size': 1, 'ram': True},
    'P_GAIN':                     {'address': 0x1C, '# data': 1, 'size': 1, 'ram': True},
    'GAINS':                      {'address': 0x1A, '# data': 3, 'size': 1, 'ram': True, 'components': ['D_GAINS', 'I_GAINS', 'P_GAINS']},
    # AX RX series
    'CW_COMPLIANCE_MARGIN':       {'address': 0x1A, '# data': 1, 'size': 1, 'ram': True},
    'CCW_COMPLIANCE_MARGIN':      {'address': 0x1B, '# data': 1, 'size': 1, 'ram': True},
    'COMPLIANCE_MARGINS':         {'address': 0x1A, '# data': 2, 'size': 1, 'ram': True, 'components': ['CW_COMPLIANCE_MARGIN', 'CCW_COMPLIANCE_MARGIN']},
    'CW_COMPLIANCE_SLOPE':        {'address': 0x1C, '# data': 1, 'size': 1, 'ram': True},
    'CCW_COMPLIANCE_SLOPE':       {'address': 0x1D, '# data': 1, 'size': 1, 'ram': True},
    'COMPLIANCE_SLOPES':          {'address': 0x1C, '# data': 2, 'size': 1, 'ram': True, 'components': ['CW_COMPLIANCE_SLOPE', 'CCW_COMPLIANCE_SLOPE']},

    'GOAL_POSITION':              {'address': 0x1E, '# data': 1, 'size': 2, 'ram': True},
    'MOVING_SPEED':               {'address': 0x20, '# data': 1, 'size': 2, 'ram': True},
    'TORQUE_LIMIT':               {'address': 0x22, '# data': 1, 'size': 2, 'ram': True},
    'GOAL_POS_SPEED_TORQUE':      {'address': 0x1E, '# data': 3, 'size': 2, 'ram': True, 'components': ['GOAL_POSITION', 'MOVING_SPEED', 'TORQUE_LIMIT']},

    'PRESENT_POSITION':           {'address': 0x24, '# data': 1, 'size': 2, 'ram': True},
    'PRESENT_POS_SPEED_LOAD':     {'address': 0x24, '# data': 3, 'size': 2, 'ram': True, 'components': ['PRESENT_POSITION', 'PRESENT_SPEED', 'PRESENT_LOAD']},
    'PRESENT_SPEED':              {'address': 0x26, '# data': 1, 'size': 2, 'ram': True},
    'PRESENT_LOAD':               {'address': 0x28, '# data': 1, 'size': 2, 'ram': True},
    'PRESENT_VOLTAGE':            {'address': 0x2A, '# data': 1, 'size': 1, 'ram': True},
    'PRESENT_TEMPERATURE':        {'address': 0x2B, '# data': 1, 'size': 1, 'ram': True},
    'REGISTERED':                 {'address': 0x2C, '# data': 1, 'size': 1, 'ram': True},
    'MOVING':                     {'address': 0x2E, '# data': 1, 'size': 1, 'ram': True},
    'LOCK':                       {'address': 0x2F, '# data': 1, 'size': 1, 'ram': True},
    'PUNCH':                      {'address': 0x30, '# data': 1, 'size': 2, 'ram': True},

    'SENSED_CURRENT':             {'address': 0x38, '# data': 1, 'size': 2, 'ram': True}, # EX only
    'CURRENT':                    {'address': 0x44, '# data': 1, 'size': 2, 'ram': True}, # MX64 and MX106

    'TORQUE_CONTROL_MODE_ENABLE': {'address': 0x46, '# data': 1, 'size': 1, 'ram': True}, # MX28 and MX106
    'GOAL_TORQUE':                {'address': 0x47, '# data': 1, 'size': 2, 'ram': True}, # MX28 and MX106
    'GOAL_ACCELERATION':          {'address': 0x49, '# data': 1, 'size': 1, 'ram': True}, # MX28 and MX106
}

def REG_ADDRESS(control_name):
    return DXL_CONTROLS[control_name]['address']

def REG_SIZE(control_name):
    return DXL_CONTROLS[control_name]['size']

def REG_LENGTH(control_name):
    return DXL_CONTROLS[control_name]['# data'] * REG_SIZE(control_name)

def REG_DATA(control_name):
    return DXL_CONTROLS[control_name]['# data']

def REG_COMPONENTS(control_name):
    return DXL_CONTROLS[control_name].get('components', None)

def REG_RAM(control_name):
    return DXL_CONTROLS[control_name]['ram']


# EEPROM
DXL_MODEL_NUMBER               = REG_ADDRESS('MODEL_NUMBER')
DXL_VERSION                    = REG_ADDRESS('VERSION')
DXL_ID                         = REG_ADDRESS('ID')
DXL_BAUD_RATE                  = REG_ADDRESS('BAUD_RATE')
DXL_RETURN_DELAY_TIME          = REG_ADDRESS('RETURN_DELAY_TIME')
DXL_CW_ANGLE_LIMIT             = REG_ADDRESS('CW_ANGLE_LIMIT')
DXL_CCW_ANGLE_LIMIT            = REG_ADDRESS('CCW_ANGLE_LIMIT')
DXL_DRIVE_MODE                 = REG_ADDRESS('DRIVE_MODE')
DXL_HIGHEST_LIMIT_TEMPERATURE  = REG_ADDRESS('HIGHEST_LIMIT_TEMPERATURE')
DXL_LOWEST_LIMIT_VOLTAGE       = REG_ADDRESS('LOWEST_LIMIT_VOLTAGE')

DXL_HIGHEST_LIMIT_VOLTAGE      = REG_ADDRESS('HIGHEST_LIMIT_VOLTAGE')
DXL_MAX_TORQUE                 = REG_ADDRESS('MAX_TORQUE')
DXL_STATUS_RETURN_LEVEL        = REG_ADDRESS('STATUS_RETURN_LEVEL')
DXL_ALARM_LED                  = REG_ADDRESS('ALARM_LED')
DXL_ALARM_SHUTDOWN             = REG_ADDRESS('ALARM_SHUTDOWN')

# RAM
DXL_TORQUE_ENABLE              = REG_ADDRESS('TORQUE_ENABLE')
DXL_LED                        = REG_ADDRESS('LED')
# RX series
DXL_CW_COMPLIANCE_MARGIN       = REG_ADDRESS('CW_COMPLIANCE_MARGIN')
DXL_CCW_COMPLIANCE_MARGIN      = REG_ADDRESS('CCW_COMPLIANCE_MARGIN')
DXL_CW_COMPLIANCE_SLOPE        = REG_ADDRESS('CW_COMPLIANCE_SLOPE')
DXL_CCW_COMPLIANCE_SLOPE       = REG_ADDRESS('CCW_COMPLIANCE_SLOPE')
# MX series
DXL_P_GAIN                     = REG_ADDRESS('P_GAIN')
DXL_I_GAIN                     = REG_ADDRESS('I_GAIN')
DXL_D_GAIN                     = REG_ADDRESS('D_GAIN')

DXL_GOAL_POSITION              = REG_ADDRESS('GOAL_POSITION')
DXL_MOVING_SPEED               = REG_ADDRESS('MOVING_SPEED')
DXL_TORQUE_LIMIT               = REG_ADDRESS('TORQUE_LIMIT')
DXL_PRESENT_POSITION           = REG_ADDRESS('PRESENT_POSITION')
DXL_PRESENT_SPEED              = REG_ADDRESS('PRESENT_SPEED')
DXL_PRESENT_LOAD               = REG_ADDRESS('PRESENT_LOAD')
DXL_PRESENT_VOLTAGE            = REG_ADDRESS('PRESENT_VOLTAGE')
DXL_PRESENT_TEMPERATURE        = REG_ADDRESS('PRESENT_TEMPERATURE')
DXL_REGISTERED                 = REG_ADDRESS('REGISTERED')
DXL_MOVING                     = REG_ADDRESS('MOVING')
DXL_LOCK                       = REG_ADDRESS('LOCK')
DXL_PUNCH                      = REG_ADDRESS('PUNCH')

# EX series
DXL_SENSED_CURRENT             = REG_ADDRESS('SENSED_CURRENT')

# MX series
DXL_CURRENT                    = REG_ADDRESS('CURRENT')

DXL_TORQUE_CONTROL_MODE_ENABLE = REG_ADDRESS('TORQUE_CONTROL_MODE_ENABLE')
DXL_GOAL_TORQUE                = REG_ADDRESS('GOAL_TORQUE')
DXL_GOAL_ACCELERATION          = REG_ADDRESS('GOAL_ACCELERATION')


# RAM bundles
DXL_ANGLE_LIMITS               = REG_ADDRESS('ANGLE_LIMITS')
DXL_PRESENT_POS_SPEED_LOAD     = REG_ADDRESS('PRESENT_POS_SPEED_LOAD')
DXL_GOAL_POS_SPEED_TORQUE      = REG_ADDRESS('GOAL_POS_SPEED_TORQUE')
DXL_COMPLIANCE_SLOPES          = REG_ADDRESS('COMPLIANCE_SLOPES')
DXL_COMPLIANCE_MARGINS         = REG_ADDRESS('COMPLIANCE_MARGINS')
DXL_GAINS                      = REG_ADDRESS('GAINS')
DXL_VOLTAGE_LIMITS             = REG_ADDRESS('VOLTAGE_LIMITS')


DXL_MODELS = {
    12:  'AX-12',
    18:  'AX-18',
    44:  'AX-12W',

    10:  'RX-10',
    24:  'RX-24F',
    28:  'RX-28',
    64:  'RX-64',

    29:  'MX-28',
    54:  'MX-64',
    320: 'MX-106',

    107: 'EX-106+',

    10028: 'VX-28', # VREP
    10064: 'VX-64', # VREP
}


