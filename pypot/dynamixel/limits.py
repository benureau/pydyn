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
        

position_range = {
    'EX' : (4095.0, 250.92),
    'MX' : (4095.0, 360.0),
    'AX' : (1023.0, 300.0),
    'RX' : (1023.0, 300.0),
}

