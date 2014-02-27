"""
This module hosts all the functions that check that user input is in range.

Once a function (eg. for motor id) is defined here, it is accessible as:
>>> limits.CHECK[pt.ID](val)
This function will raise ValueError if val is not between 0 and 253.

Functions were written by perusing the english documentation, mostly of the AX,
RX, and MX servos:
    http://support.robotis.com/en/product/dynamixel/dxl_ax_main.htm
    http://support.robotis.com/en/product/dynamixel/dxl_rx_main.htm
    http://support.robotis.com/en/product/dynamixel/dxl_mx_main.htm
We might have overlooked cases specific to other servos, don't hesitate to report an
issue on github.
"""


import numbers

from . import protocol as pt

POSITION_RANGES = {
    'EX' : (4095, 250.92),
    'MX' : (4095, 360.0),
    'AX' : (1023, 300.0),
    'RX' : (1023, 300.0),
    'VX' : (1023, 300.0),
}





# MARK Bound functions

def checkbounds(name, lower, upper, val):
    if not lower <= val <= upper:
        raise ValueError('{} should be in the [{}, {}] range but is {}'.format(name, lower, upper, val))

def checkbounds_warning(name, lower, upper, val):
    legal_val = max(lower, min(upper, val))
    if legal_val != val:
        raise Warning('{} should be in the [{}, {}] range but is {}'.format(name, lower, upper, val))
    return legal_val

def checkbounds_mode(name, lower, upper, val, mode):
    if not lower <= val <= upper:
        raise ValueError('in {} mode, {} should be in the [{}, {}] range but is {}'.format(mode, name, lower, upper, val))

def checkoneof(name, collection, val):
    if not val in collection:
        raise ValueError('{} should be in {} but is {}'.format(name, collection, val))



def def_bounds_unit(lower, upper, desc, unit):
    def _bounds(val):
        if not lower <= val <= upper:
           raise ValueError('value of {} is {} {}, but should be between {} and {}'.format(name, val, unit, lower, upper))
    return _bounds

def def_bounds_int(lower, upper, desc):
    def _bounds(val):
        if not lower <= val <= upper:
            raise ValueError('{} value is {}, but should be between {} and {}'.format(name, val, lower, upper))
        if not isinstance(val, numbers.Integral):
            raise ValueError('{} raw value ({}) should be an integer'.format(val))
    return _bounds

def def_oneof_int(choices, desc):
    def _bounds(val):
        if not val in choices:
            raise ValueError('{} value is {}, but should be one of {}'.format(name, val, choices))
        if not isinstance(val, numbers.Integral):
            raise ValueError('{} raw value ({}) should be an integer'.format(val))
    return _bounds


CHECK = {
    pt.ID                        : def_bounds_int(  0,  253, 'id'),
    pt.BAUD_RATE                 : def_bounds_int(  0,  254, 'baudrate (bytes)'),
    pt.RETURN_DELAY_TIME         : def_bounds_int(  0,  254, 'return delay time (bytes)'),
    pt.HIGHEST_LIMIT_TEMPERATURE : def_bounds_int( 10,   99, 'max temperature (bytes)'),
    pt.HIGHEST_LIMIT_VOLTAGE     : def_bounds_int( 50,  250, 'max voltage (bytes)'),
    pt.LOWEST_LIMIT_VOLTAGE      : def_bounds_int( 50,  250, 'min voltage (bytes)'),
    pt.MAX_TORQUE                : def_bounds_int(  0, 1023, 'max torque (bytes)'),
    pt.STATUS_RETURN_LEVEL       : def_oneof_int((0, 1, 2),  'status return level (bytes)'),
}
