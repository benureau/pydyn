import os, sys
import glob

import color

from pypot.dynamixel.io import DynamixelIO
from pypot.dynamixel.controller import DynamixelController

def get_available_ports():
    """ 
        Returns the available ports found.
        
        The available ports are looked for in the following path depending on the OS:
            * Linux : /dev/ttyACM* and /dev/ttyUSB*
            * Mac OS : /dev/tty.usb*
        
        :raises: NotImplementedError if your OS is not one of the currently supported (Linux, Mac OS).
        
        """
    op_system = os.uname()[0]
    
    if op_system == 'Darwin':
        return glob.glob('/dev/tty.usb*')
    
    elif op_system == 'Linux':
        return glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')

    raise NotImplementedError('Unknown operating system: %s' % (op_system))

def create_controller(connection_type="USB2DXL", verbose = False, motor_range = None, timeout = 0.02):
    """
        Return a controller outfitted with all motor found.
    """
    if verbose:
        print 'Loading the robot...'
    ports = get_available_ports()
    if not ports:
        if verbose:
            print '  [{}FAIL{}] No standart port found. If you port has a funny name, you might want to specify it explicetly.'.format(color.red, color.end)
        print 'Error; exiting.'
        exit(1)
    
    port = ports[0]
    if verbose:
        print '  [{} OK {}] Port found : {}{}{}'.format(color.green, color.end, color.bold, port, color.end)
    ctrl = DynamixelController(port, connection_type, timeout = timeout)
    if verbose:
        print '  [{} OK {}] Connexion established : {}'.format(color.green, color.end, ctrl.io)
    
    if motor_range is None:
        motor_range = 0, 253
    # if verbose:
    #     print '  Scanning motors ids between {} and {}'.format(motor_range[0], motor_range[-1])
    
    motor_ids = ctrl.discover_motors(range(motor_range[0],motor_range[-1]+1), verbose = verbose)
    if verbose:
        print '  [{} OK {}] Scanning motor ids between {} and {}             '.format(color.green,
                color.end, motor_ids[0], motor_ids[-1])

    if len(motor_ids) == 0:
        print '  [{}FAIL{}] No motor found. Verify connections, power, USB dongle state.'.format(color.red,
        color.end, motor_ids[0], motor_ids[-1])
        print 'Error; exiting.'
        exit(0)
    else:
        if verbose:
            print '  [{} OK {}] Motors found : {}'.format(color.green, color.end, ', '.join('{}{}{}'.format(color.cyan, m_id, color.end) for m_id in motor_ids))
        
    if verbose:
        print '  [{}LOAD{}] Loading EEPROMs and RAMs...\r'.format(color.iblue, color.end),
        sys.stdout.flush()
    motors = ctrl.load_motors(motor_ids, load_eeprom = True)    
    
    if verbose:
        print '  [{} OK {}] Loaded EEPROMs and RAMs     '.format(color.green, color.end)
        print '  [{} OK {}] Models : {}'.format(color.green, color.end, 
                ', '.join(['{}{}{}'.format(color.bold, m.model, color.end) for m in motors]))

    ctrl.start_sync()
    
    return ctrl
