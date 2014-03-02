"""
This module hosts all the functions that check that user input is in range.

Once a function (eg. for motor id) is defined here, it is accessible as:

    limits.CHECK[pt.ID](val)

This function will raise ValueError if value is not between 0 and 253.

Some function need the modelclass and the mode of the motor to function
properly. For convenience, every checking function accepts those arguments:

    limits.CHECK[pt.MAX_TORQUE](value, modelclass=None, mode=None)

Functions were written by perusing the english documentation, of the AX, DX, RX,
EX, and MX servos:
    http://support.robotis.com/en/product/dynamixel/dxl_ax_main.htm
    http://support.robotis.com/en/product/dynamixel/dxl_dx_main.htm
    http://support.robotis.com/en/product/dynamixel/dxl_rx_main.htm
    http://support.robotis.com/en/product/dynamixel/dxl_ex_main.htm
    http://support.robotis.com/en/product/dynamixel/dxl_mx_main.htm
We might have overlooked cases specific to some models, don't hesitate to report an
issue on github.
"""
from __future__ import division

import numbers

from . import protocol as pt

POSITION_RANGES = {
         # bytes, degree
    'EX' : (4095, 250.92),
    'MX' : (4095, 360.0),
    'AX' : (1023, 300.0),
    'DX' : (1023, 300.0),
    'RX' : (1023, 300.0),
    'VX' : (1023, 300.0),
}


SPEED_RANGES = {
         # bytes, degree/s
    'AX' : (1023, 1023*6*0.111),
    'DX' : (1023, 1023*6*0.111),
    'RX' : (1023, 1023*6*0.111),
    'EX' : (1023, 1023*6*0.111),
    'MX' : (1023, 1023*6*0.11445),
    'VX' : (1023, 1023*6*0.111),
}

# minimum
PUNCH_RANGES = {
         # bytes, degree
    'EX' : (  32, 32/1023.0),
    'MX' : (   0, 0.0),
    'AX' : (  32, 32/1023.0),
    'DX' : (  32, 32/1023.0),
    'RX' : (  32, 32/1023.0),
    'VX' : (  32, 32/1023.0),
}

MAX_P_GAIN = 254.0 / 8.0
MAX_I_GAIN = 254.0 * 1000.0 / 2048.0
MAX_D_GAIN = 254.0 * 4.0 / 1000.0

MIN_CURRENT = -2048 * 0.0045 #TODO: 2047 ?
MAX_CURRENT =  2048 * 0.0045

MIN_SENSED_CURRENT = -512 * 0.01 #TODO: 511 ?
MAX_SENSED_CURRENT =  512 * 0.01


# MARK Bound functions

def checkbounds(name, lower, upper, value):
    if not lower <= value <= upper:
        raise ValueError('{} should be in the [{}, {}] range but is {}'.format(name, lower, upper, value))

def checkbounds_warning(name, lower, upper, value):
    if not lower <= value <= upper:
        raise Warning('{} should be in the [{}, {}] range but is {}'.format(name, lower, upper, value))

def checkbounds_mode(name, lower, upper, value, mode):
    if not lower <= value <= upper:
        raise ValueError('in {} mode, {} should be in the [{}, {}] range but is {}'.format(mode, name, lower, upper, value))

def checkoneof(name, collection, value):
    if not value in collection:
        raise ValueError('{} should be in {} but is {}'.format(name, collection, value))


def _check_bytes(value, desc):
    if not isinstance(value, numbers.Integral):
        raise ValueError('{} value ({}) should be an integer.'.format(desc, value))

def _def_bounds(lower, upper, desc):
    def _bounds(value, modelclass=None, mode=None):
        if not lower <= value <= upper:
            raise ValueError('value of {} is {}, but should be between {} and {}.'.format(desc, value, lower, upper))
    return _bounds

def _def_bounds_unit(lower, upper, desc, unit):
    def _bounds(value, modelclass=None, mode=None):
        if not lower <= value <= upper:
            raise ValueError('value of {} is {} {}, but should be between {} and {}.'.format(desc, value, unit, lower, upper))
    return _bounds

def _def_bounds_bytes(lower, upper, desc):
    def _bounds(value, modelclass=None, mode=None):
        _check_bytes(value, desc)
        if not lower <= value <= upper:
            raise ValueError('{} value is {}, but should be between {} and {}.'.format(desc, value, lower, upper))
    return _bounds

def _def_oneof_bytes(choices, desc):
    def _bounds(value, modelclass=None, mode=None):
        _check_bytes(value, desc)
        if not value in choices:
            raise ValueError('{} value is {}, but should be one of {}.'.format(desc, value, choices))
    return _bounds

def _def_chain_bytes(control):
    assert len(control.parts) > 1
    def _bounds(values, modelclass=None, mode=None):
        if len(control.parts) != len(values):
            raise ValueError('{} requires {} values, but {} was provided.'.format(control.name, len(control.parts), values))
        for value, c in zip(values, control.parts):
            CHECK_BYTES[c](value, modelclass=modelclass, mode=mode)
    return _bounds

def _def_chain(control):
    assert len(control.parts) > 1
    def _bounds(values, modelclass=None, mode=None):
        if len(control.parts) != len(values):
            raise ValueError('{} requires {} values, but {} was provided.'.format(control.name, len(control.parts), values))
        for value, c in zip(values, control.parts):
            CHECK[c](value, modelclass=modelclass, mode=mode)
    return _bounds


def _no_checks(value, modelclass=None, mode=None):
    pass

def _def_position_bytes(desc):
    def _bounds(value, modelclass=None, mode=None):
        _check_bytes(value, desc)
        max_pos = POSITION_RANGES[modelclass][0]
        if not 0 <= value <= max_pos:
            raise ValueError('{} value is {}, but should be between {} and {}.'.format(desc, value, 0.0, max_pos))
    return _bounds

def _def_position(desc):
    def _bounds(value, modelclass=None, mode=None):
        max_pos = POSITION_RANGES[modelclass][1]/2.0
        if not -max_pos <= value <= max_pos:
            raise ValueError('{} value is {} degrees, but should be between {} and {} degrees.'.format(desc, value, -max_pos, max_pos))
    return _bounds

def _def_speed_bytes(desc):
    def _bounds(value, modelclass=None, mode=None):
        _check_bytes(value, desc)
        max_speed = SPEED_RANGES[modelclass][0]
        if mode == 'joint':
            if not 0 <= value <= 1023:
                raise ValueError('{} value is {}, but should be between 0 and 1023 (joint mode).'.format(desc, value))
        else:
            if not 0 <= value <= 2047:
                raise ValueError('{} value is {}, but should be between 0 and 2047 (wheel mode).'.format(desc, value))
    return _bounds

def _moving_speed(value, modelclass=None, mode=None):
    max_speed = SPEED_RANGES[modelclass][1]
    if mode == 'joint':
        if not 0.0 <= value <= max_speed:
            raise ValueError('moving speed value is {} %, but should be between 0.0 and {} % (joint mode).'.format(value, max_speed))
    else:
        if modelclass != 'MX' and mode == 'wheel':
            if not -100.0 <= value <= 100.0:
                raise ValueError('speed value is {} %, but should be between -100.0 and 100.0 % (wheel mode).'.format(value))
        else:
            max_speed = SPEED_RANGES[modelclass][1]
            if not -max_speed <= value <= max_speed:
                raise ValueError('speed value is {} degree per second, but should be between {} and {} degree per second.'.format(value, -max_speed, max_speed))

def _present_speed(value, modelclass=None, mode=None):
    if modelclass != 'MX' and mode == 'wheel':
        if not -100.0 <= value <= 100.0:
            raise ValueError('speed value is {} %, but should be between -100.0 and 100.0 % (wheel mode).'.format(value))
    else:
        max_speed = SPEED_RANGES[modelclass][1]
        if not -max_speed <= value <= max_speed:
            raise ValueError('speed value is {} degree per second, but should be between {} and {} degrees.'.format(value, -max_speed, max_speed))

def _punch(value, modelclass=None, mode=None):
    if not PUNCH_RANGES[modelclass][1] <= value <= 100.0:
        raise ValueError('punch value is {} %, but should be between {} and {} %.'.format(value, PUNCH_RANGES[modelclass][1], 100.0))

def _punch_bytes(value, modelclass=None, mode=None):
    _check_bytes(value, 'punch (bytes)')
    if not PUNCH_RANGES[modelclass][0] <= value <= 1023:
        raise ValueError('punch (bytes) value is {}, but should be between {} and {}.'.format(value, PUNCH_RANGES[modelclass][0], 1023))

def _def_compliance_margin(desc):
    def _bounds(value, modelclass=None, mode=None):
        max_pos, max_deg = POSITION_RANGES[modelclass]
        max_margin = max_deg*255/max_pos
        if 0.0 <= value <= max_margin:
            raise ValueError('{} value is {} degree, but should be between {} and {}'.format(desc, value, 0.0, max_margin))
    return _bounds

CHECK_BYTES = {
    pt.MODEL_NUMBER              : _def_bounds_bytes(  0, 4095, 'firware version (bytes)'),
    pt.FIRMWARE                  : _def_bounds_bytes(  0,  253, 'firware version (bytes)'),
    pt.ID                        : _def_bounds_bytes(  0,  253, 'id (bytes)'),
    pt.BAUDRATE                  : _def_bounds_bytes(  0,  254, 'baudrate (bytes)'),
    pt.RETURN_DELAY_TIME         : _def_bounds_bytes(  0,  254, 'return delay time (bytes)'),
    pt.CW_ANGLE_LIMIT            : _def_position_bytes(         'cw angle limit (bytes)'),
    pt.CCW_ANGLE_LIMIT           : _def_position_bytes(         'ccw angle limit (bytes)'),
    pt.DRIVE_MODE                : _def_bounds_bytes(  0,  255, 'drive mode (bytes)'),
    pt.HIGHEST_LIMIT_TEMPERATURE : _def_bounds_bytes( 10,   99, 'max temperature (bytes)'),
    pt.HIGHEST_LIMIT_VOLTAGE     : _def_bounds_bytes( 50,  250, 'max voltage (bytes)'),
    pt.LOWEST_LIMIT_VOLTAGE      : _def_bounds_bytes( 50,  250, 'min voltage (bytes)'),
    pt.MAX_TORQUE                : _def_bounds_bytes(  0, 1023, 'max torque (bytes)'),
    pt.STATUS_RETURN_LEVEL       : _def_oneof_bytes( (0, 1, 2), 'status return level (bytes)'),
    pt.ALARM_LED                 : _def_bounds_bytes(  0,  255, 'alarm led (bytes)'),
    pt.ALARM_SHUTDOWN            : _def_bounds_bytes(  0,  255, 'alarm shutdown (bytes)'),

    # RAM
    pt.TORQUE_ENABLE             : _def_oneof_bytes(    [0, 1], 'torque_enable (bytes)'),
    pt.LED                       : _def_oneof_bytes(    [0, 1], 'torque_enable (bytes)'),
    pt.CW_COMPLIANCE_MARGIN      : _def_bounds_bytes(  0,  255, 'cw compliance margin (bytes)'),
    pt.CCW_COMPLIANCE_MARGIN     : _def_bounds_bytes(  0,  255, 'ccw compliance margin (bytes)'),
    pt.COMPLIANCE_MARGINS        : _def_chain_bytes(pt.COMPLIANCE_MARGINS),
    pt.CW_COMPLIANCE_SLOPE       : _def_bounds_bytes(  0,  255, 'cw compliance scope (bytes)'),
    pt.CCW_COMPLIANCE_SLOPE      : _def_bounds_bytes(  0,  255, 'ccw compliance scope (bytes)'),
    pt.COMPLIANCE_SLOPES         : _def_chain_bytes(pt.COMPLIANCE_SLOPES),
    pt.D_GAIN                    : _def_bounds_bytes(  0,  254, 'd gain (bytes)'),
    pt.I_GAIN                    : _def_bounds_bytes(  0,  254, 'i gain (bytes)'),
    pt.P_GAIN                    : _def_bounds_bytes(  0,  254, 'p gain (bytes)'),
    pt.GAINS                     : _def_chain_bytes(pt.GAINS),

    pt.GOAL_POSITION             : _def_position_bytes(         'goal position (bytes)'),
    pt.MOVING_SPEED              : _def_speed_bytes(            'moving speed (bytes)'),
    pt.TORQUE_LIMIT              : _def_bounds_bytes(  0, 1023, 'torque limit (bytes)'),
    pt.PRESENT_POSITION          : _def_position_bytes(         'present position (bytes)'),
    pt.PRESENT_SPEED             : _def_bounds_bytes(  0, 2047, 'present load (bytes)'),
    pt.PRESENT_LOAD              : _def_bounds_bytes(  0, 2047, 'present load (bytes)'),
    pt.PRESENT_VOLTAGE           : _def_bounds_bytes(  0,  254, 'present voltage (bytes)'),
    pt.PRESENT_TEMPERATURE       : _def_bounds_bytes(  0,  254, 'present temperature (bytes)'),
    pt.REGISTERED                : _def_oneof_bytes(    [0, 1], 'registered (bytes)'),
    pt.MOVING                    : _def_oneof_bytes(    [0, 1], 'moving (bytes)'),
    pt.LOCK                      : _def_oneof_bytes(    [0, 1], 'lock (bytes)'),
    pt.PUNCH                     : _punch_bytes,

    pt.CURRENT                   : _def_bounds_bytes(  0, 4095, 'current (bytes)'),
    pt.SENSED_CURRENT            : _def_bounds_bytes(  0, 1024, 'sensed current (bytes)'),

    pt.TORQUE_CONTROL_MODE_ENABLE: _def_oneof_bytes(    [0, 1], 'torque control mode enable (bytes)'),
    pt.GOAL_TORQUE               : _def_bounds_bytes(  0, 2047, 'goal torque (bytes)'),
    pt.GOAL_ACCELERATION         : _def_bounds_bytes(  0,  254, 'goal acceleration (bytes)'),
}

CHECK = {
    pt.MODEL_NUMBER              : _def_bounds_bytes(  0, 4095, 'model number'),
    pt.FIRMWARE                  : _def_bounds_bytes(  0,  253, 'firware version'),
    pt.ID                        : _def_bounds_bytes(  0,  253, 'id '),
    pt.BAUDRATE                  : _def_bounds_unit(7874, 1000000, 'baudrate', 'bps'), # FIXME for MX
    pt.RETURN_DELAY_TIME         : _def_bounds_unit(   0,  508, 'return delay time', 'us'),
    pt.CW_ANGLE_LIMIT            : _def_position(               'cw angle limit'),
    pt.CCW_ANGLE_LIMIT           : _def_position(               'ccw angle limit'),
    pt.DRIVE_MODE                : _no_checks,
    pt.HIGHEST_LIMIT_TEMPERATURE : _def_bounds_unit(  10,   99, 'max temperature', 'degrees'),
    pt.HIGHEST_LIMIT_VOLTAGE     : _def_bounds_unit(   5,   25, 'max voltage', 'V'),
    pt.LOWEST_LIMIT_VOLTAGE      : _def_bounds_unit(   5,   25, 'min voltage', 'V'),
    pt.MAX_TORQUE                : _def_bounds_unit(   0,  100, 'max torque',  '%'),
    pt.STATUS_RETURN_LEVEL       : _def_oneof_bytes(( 0, 1, 2), 'status return level'),
    pt.ALARM_LED                 : _no_checks, # TODO
    pt.ALARM_SHUTDOWN            : _no_checks, # TODO

    # RAM
    pt.TORQUE_ENABLE             : _no_checks,
    pt.LED                       : _no_checks,
    pt.CW_COMPLIANCE_MARGIN      : _def_compliance_margin(      'cw compliance margin'),
    pt.CCW_COMPLIANCE_MARGIN     : _def_compliance_margin(      'ccw compliance margin'),
    pt.COMPLIANCE_MARGINS        : _def_chain_bytes(pt.COMPLIANCE_MARGINS),
    pt.CW_COMPLIANCE_SLOPE       : _def_oneof_bytes(  range(8), 'cw compliance scope'),
    pt.CCW_COMPLIANCE_SLOPE      : _def_oneof_bytes(  range(8), 'ccw compliance scope'),
    pt.COMPLIANCE_SLOPES         : _def_chain_bytes(pt.COMPLIANCE_SLOPES),
    pt.D_GAIN                    : _def_bounds(  0, MAX_D_GAIN, 'd gain'),
    pt.I_GAIN                    : _def_bounds(  0, MAX_I_GAIN, 'i gain'),
    pt.P_GAIN                    : _def_bounds(  0, MAX_P_GAIN, 'p gain'),
    pt.GAINS                     : _def_chain(pt.GAINS),

    pt.GOAL_POSITION             : _def_position(               'goal position'),
    pt.MOVING_SPEED              : _moving_speed,
    pt.TORQUE_LIMIT              : _def_bounds_unit(   0,  100, 'torque limit',  '%'),
    pt.PRESENT_POSITION          : _def_position(               'present position'),
    pt.PRESENT_SPEED             : _present_speed,
    pt.PRESENT_LOAD              : _def_bounds(     -100,  100, 'present load'),
    pt.PRESENT_VOLTAGE           : _def_bounds_unit(   0, 25.5, 'present voltage', 'V'),
    pt.PRESENT_TEMPERATURE       : _def_bounds_unit(   0,  255, 'present voltage', 'V'),
    pt.REGISTERED                : _no_checks,
    pt.MOVING                    : _no_checks,
    pt.LOCK                      : _no_checks,
    pt.PUNCH                     : _punch,

    pt.CURRENT                   : _def_bounds_unit(MIN_CURRENT, MAX_CURRENT, 'current', 'A'),
    pt.SENSED_CURRENT            : _def_bounds_unit(MIN_SENSED_CURRENT, MAX_SENSED_CURRENT, 'sensed current', 'A'),

    pt.TORQUE_CONTROL_MODE_ENABLE: _no_checks,
    pt.GOAL_TORQUE               : _def_bounds_unit(   0,  100, 'goal torque',  '%'),
    pt.GOAL_ACCELERATION         : _def_bounds_bytes(0,  254*8.583, 'goal acceleration (bytes)'),

}

assert set(CHECK.keys()) == set(CHECK_BYTES.keys())
