# For details on the dynamixel protocol you should refer to
# http://support.robotis.com/en/product/dxl_main.htm

import collections

PING       = 0x01
READ_DATA  = 0x02
WRITE_DATA = 0x03
REG_WRITE  = 0x04
ACTION     = 0x05
RESET      = 0x06
SYNC_WRITE = 0x83
SYNC_READ  = 0x84

BROADCAST = 254


INPUT_VOLTAGE_ERROR = 0
ANGLE_LIMIT_ERROR   = 1
OVERHEATING_ERROR   = 2
RANGE_ERROR         = 3
CHECKSUM_ERROR      = 4
OVERLOAD_ERROR      = 5
INSTRUCTION_ERROR   = 6


""" Control namedtuple
:param name:     the name of the control
:param addr:  the starting addr of the controls
:param sizes:    a list of the size (1 or 2) of each memory cell of the controls
:param ram:      True if in RAM, False if EEPROM
:param models:   The model that support this control
"""
Control = collections.namedtuple('Control', ['name', 'addr', 'sizes', 'ram', 'models'])

MODELS = {
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

POSITION_RANGES = {
    'EX' : (4095.0, 250.92),
    'MX' : (4095.0, 360.0),
    'AX' : (1023.0, 300.0),
    'RX' : (1023.0, 300.0),
    'VX' : (1023.0, 300.0),
}

_all_models   = frozenset(MODELS.values())
_mx_models    = frozenset(('MX-12', 'MX-28', 'MX-64', 'MX-106'))
_axrx_models  = frozenset(('AX-12', 'AX-18', 'AX-12W', 'RX-10', 'RX-24F', 'RX-28', 'RX-64'))

# this list is in sorted by starting addr
CTRL_LIST = [
    # EEPROM
    Control(name='MODEL_NUMBER',               addr= 0, sizes=(2,),    ram=False, models=_all_models),
    Control(name='VERSION',                    addr= 2, sizes=(1,),    ram=False, models=_all_models),
    Control(name='ID',                         addr= 3, sizes=(1,),    ram=False, models=_all_models),
    Control(name='BAUD_RATE',                  addr= 4, sizes=(1,),    ram=False, models=_all_models),
    Control(name='RETURN_DELAY_TIME',          addr= 5, sizes=(1,),    ram=False, models=_all_models),
    Control(name='CW_ANGLE_LIMIT',             addr= 6, sizes=(2,),    ram=False, models=_all_models),
    Control(name='CCW_ANGLE_LIMIT',            addr= 8, sizes=(2,),    ram=False, models=_all_models),
    Control(name='DRIVE_MODE',                 addr=10, sizes=(1,),    ram=False, models=set('EX-106+')),
    Control(name='HIGHEST_LIMIT_TEMPERATURE',  addr=11, sizes=(1,),    ram=False, models=_all_models),
    Control(name='LOWEST_LIMIT_VOLTAGE',       addr=12, sizes=(1,),    ram=False, models=_all_models),
    Control(name='HIGHEST_LIMIT_VOLTAGE',      addr=13, sizes=(1,),    ram=False, models=_all_models),
    Control(name='MAX_TORQUE',                 addr=14, sizes=(2,),    ram=False, models=_all_models),
    Control(name='STATUS_RETURN_LEVEL',        addr=16, sizes=(1,),    ram=False, models=_all_models),
    Control(name='ALARM_LED',                  addr=17, sizes=(1,),    ram=False, models=_all_models),
    Control(name='ALARM_SHUTDOWN',             addr=18, sizes=(1,),    ram=False, models=_all_models),

    #RAM
    Control(name='TORQUE_ENABLE',              addr=24, sizes=(1,),    ram=True, models=_all_models),
    Control(name='LED',                        addr=25, sizes=(1,),    ram=True, models=_all_models),

    Control(name='D_GAIN',                     addr=26, sizes=(1,),    ram=True, models=_mx_models),
    Control(name='I_GAIN',                     addr=27, sizes=(1,),    ram=True, models=_mx_models),
    Control(name='P_GAIN',                     addr=28, sizes=(1,),    ram=True, models=_mx_models),

    Control(name='CW_COMPLIANCE_MARGIN',       addr=26, sizes=(1,),    ram=True, models=_axrx_models),
    Control(name='CCW_COMPLIANCE_MARGIN',      addr=27, sizes=(1,),    ram=True, models=_axrx_models),
    Control(name='CW_COMPLIANCE_SLOPE',        addr=28, sizes=(1,),    ram=True, models=_axrx_models),
    Control(name='CCW_COMPLIANCE_SLOPE',       addr=29, sizes=(1,),    ram=True, models=_axrx_models),

    Control(name='GOAL_POSITION',              addr=30, sizes=(2,),    ram=True, models=_all_models),
    Control(name='MOVING_SPEED',               addr=32, sizes=(2,),    ram=True, models=_all_models),
    Control(name='TORQUE_LIMIT',               addr=34, sizes=(2,),    ram=True, models=_all_models),
    Control(name='PRESENT_POSITION',           addr=36, sizes=(2,),    ram=True, models=_all_models),
    Control(name='PRESENT_SPEED',              addr=38, sizes=(2,),    ram=True, models=_all_models),
    Control(name='PRESENT_LOAD',               addr=40, sizes=(2,),    ram=True, models=_all_models),

    Control(name='PRESENT_VOLTAGE',            addr=42, sizes=(1,),    ram=True, models=_all_models),
    Control(name='PRESENT_TEMPERATURE',        addr=43, sizes=(1,),    ram=True, models=_all_models),
    Control(name='REGISTERED',                 addr=44, sizes=(1,),    ram=True, models=_all_models),
    Control(name='MOVING',                     addr=46, sizes=(1,),    ram=True, models=_all_models),
    Control(name='LOCK',                       addr=47, sizes=(1,),    ram=True, models=_all_models),
    Control(name='PUNCH',                      addr=48, sizes=(2,),    ram=True, models=_all_models),

    Control(name='SENSED_CURRENT',             addr=56, sizes=(2,),    ram=True, models=set('EX-106+')),

    Control(name='CURRENT',                    addr=68, sizes=(2,),    ram=True, models=_mx_models),
    Control(name='TORQUE_CONTROL_MODE_ENABLE', addr=70, sizes=(1,),    ram=True, models=_mx_models),
    Control(name='GOAL_TORQUE',                addr=71, sizes=(2,),    ram=True, models=_mx_models),
    Control(name='GOAL_ACCELERATION',          addr=73, sizes=(1,),    ram=True, models=_mx_models)
]


# two functions to automatize creation of compound controls
def _memory_chunk_ctrl(name, models, start, end):
    """Create automatically a control for a part of memory"""
    assert start >= 0
    sizes_d = {}
    ram = True
    for ctrl in CTRL_LIST:
        if len(ctrl.sizes) == 1 and start <= ctrl.addr < end - ctrl.sizes[0]:
            if ctrl.addr in sizes_d:
                assert ctrl.sizes[0] == sizes_d[ctrl.addr]
            else:
                if ctrl.addr-1 in sizes_d:
                    assert sizes_d[ctrl.addr-1] == 1
                sizes_d[ctrl.addr] = ctrl.sizes[0]
            ram = ram and ctrl.ram
    sizes = []
    cursor = start
    while start <= cursor < end:
        if cursor in sizes_d:
            sizes.append(sizes_d[cursor])
            cursor += sizes_d[cursor]
        else:
            sizes.append(1)
            cursor += 1
    assert sum(sizes) == end - start
    return Control(name=name, addr=start, sizes=tuple(sizes), ram=ram, models=models)

CTRL = {ctrl.name: ctrl for ctrl in CTRL_LIST}
def _ctrl_from_ctrl_names(name, models, ctrl_names):
    """Create a compound control for multiple, consecutive controls"""
    ctrls=[CTRL[cname] for cname in ctrl_names]
    for i in range(len(ctrl_names)-1): # assert continuity
        assert ctrls[i].addr + sum(ctrls[i].sizes) == ctrls[i+1].addr

    sizes = ()
    ram = True
    for ctrl in ctrls:
        sizes += ctrl.sizes
        ram = ram and ctrl.ram
    return Control(name=name, addr=ctrls[0].addr, sizes=sizes, ram=ram, models=models)


CTRL_LIST.append(_ctrl_from_ctrl_names('VOLTAGE_LIMITS',         _all_models, ['LOWEST_LIMIT_VOLTAGE', 'HIGHEST_LIMIT_VOLTAGE']))
CTRL_LIST.append(_ctrl_from_ctrl_names('ANGLE_LIMITS',           _all_models, ['CW_ANGLE_LIMIT', 'CCW_ANGLE_LIMIT']))
CTRL_LIST.append(_ctrl_from_ctrl_names('GAINS',                   _mx_models, ['D_GAIN', 'I_GAIN', 'P_GAIN']))
CTRL_LIST.append(_ctrl_from_ctrl_names('COMPLIANCE_MARGINS',    _axrx_models, ['CW_COMPLIANCE_MARGIN', 'CCW_COMPLIANCE_MARGIN']))
CTRL_LIST.append(_ctrl_from_ctrl_names('COMPLIANCE_SLOPES',     _axrx_models, ['CW_COMPLIANCE_SLOPE', 'CCW_COMPLIANCE_SLOPE']))
CTRL_LIST.append(_ctrl_from_ctrl_names('GOAL_POS_SPEED_TORQUE',  _all_models, ['GOAL_POSITION', 'MOVING_SPEED', 'TORQUE_LIMIT']))
CTRL_LIST.append(_ctrl_from_ctrl_names('SPEED_TORQUE',           _all_models, ['MOVING_SPEED', 'TORQUE_LIMIT']))
CTRL_LIST.append(_ctrl_from_ctrl_names('PRESENT_POS_SPEED_LOAD', _all_models, ['PRESENT_POSITION', 'PRESENT_SPEED', 'PRESENT_LOAD']))

CTRL_LIST.append(_memory_chunk_ctrl('EEPROM', _all_models,  0, 24))
CTRL_LIST.append(_memory_chunk_ctrl('RAM',    _all_models, 24, 74))

CTRL = {ctrl.name: ctrl for ctrl in CTRL_LIST}
assert len(CTRL_LIST) == len(CTRL)

# EEPROM
EEPROM                     = CTRL['EEPROM']
MODEL_NUMBER               = CTRL['MODEL_NUMBER']
VERSION                    = CTRL['VERSION']
ID                         = CTRL['ID']
BAUD_RATE                  = CTRL['BAUD_RATE']
RETURN_DELAY_TIME          = CTRL['RETURN_DELAY_TIME']
CW_ANGLE_LIMIT             = CTRL['CW_ANGLE_LIMIT']
CCW_ANGLE_LIMIT            = CTRL['CCW_ANGLE_LIMIT']
DRIVE_MODE                 = CTRL['DRIVE_MODE']
HIGHEST_LIMIT_TEMPERATURE  = CTRL['HIGHEST_LIMIT_TEMPERATURE']
LOWEST_LIMIT_VOLTAGE       = CTRL['LOWEST_LIMIT_VOLTAGE']
HIGHEST_LIMIT_VOLTAGE      = CTRL['HIGHEST_LIMIT_VOLTAGE']
MAX_TORQUE                 = CTRL['MAX_TORQUE']
STATUS_RETURN_LEVEL        = CTRL['STATUS_RETURN_LEVEL']
ALARM_LED                  = CTRL['ALARM_LED']
ALARM_SHUTDOWN             = CTRL['ALARM_SHUTDOWN']

# RAM
RAM                        = CTRL['RAM']
TORQUE_ENABLE              = CTRL['TORQUE_ENABLE']
LED                        = CTRL['LED']
# RX series
CW_COMPLIANCE_MARGIN       = CTRL['CW_COMPLIANCE_MARGIN']
CCW_COMPLIANCE_MARGIN      = CTRL['CCW_COMPLIANCE_MARGIN']
CW_COMPLIANCE_SLOPE        = CTRL['CW_COMPLIANCE_SLOPE']
CCW_COMPLIANCE_SLOPE       = CTRL['CCW_COMPLIANCE_SLOPE']
# MX series
P_GAIN                     = CTRL['P_GAIN']
I_GAIN                     = CTRL['I_GAIN']
D_GAIN                     = CTRL['D_GAIN']
#
GOAL_POSITION              = CTRL['GOAL_POSITION']
MOVING_SPEED               = CTRL['MOVING_SPEED']
TORQUE_LIMIT               = CTRL['TORQUE_LIMIT']
PRESENT_POSITION           = CTRL['PRESENT_POSITION']
PRESENT_SPEED              = CTRL['PRESENT_SPEED']
PRESENT_LOAD               = CTRL['PRESENT_LOAD']
PRESENT_VOLTAGE            = CTRL['PRESENT_VOLTAGE']
PRESENT_TEMPERATURE        = CTRL['PRESENT_TEMPERATURE']
REGISTERED                 = CTRL['REGISTERED']
MOVING                     = CTRL['MOVING']
LOCK                       = CTRL['LOCK']
PUNCH                      = CTRL['PUNCH']
# EX series
SENSED_CURRENT             = CTRL['SENSED_CURRENT']
# MX series
CURRENT                    = CTRL['CURRENT']
TORQUE_CONTROL_MODE_ENABLE = CTRL['TORQUE_CONTROL_MODE_ENABLE']
GOAL_TORQUE                = CTRL['GOAL_TORQUE']
GOAL_ACCELERATION          = CTRL['GOAL_ACCELERATION']

# EEPROM bundles
ANGLE_LIMITS               = CTRL['ANGLE_LIMITS']
VOLTAGE_LIMITS             = CTRL['EEPROM']

# RAM bundles
GAINS                      = CTRL['GAINS']
COMPLIANCE_MARGINS         = CTRL['COMPLIANCE_MARGINS']
COMPLIANCE_SLOPES          = CTRL['COMPLIANCE_SLOPES']
GOAL_POS_SPEED_TORQUE      = CTRL['GOAL_POS_SPEED_TORQUE']
PRESENT_POS_SPEED_LOAD     = CTRL['PRESENT_POS_SPEED_LOAD']


