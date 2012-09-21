import os
import glob

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
        print 'Loading the platform...'
    ports = get_available_ports()
    if not ports:
        if verbose:
            print '  No port found :-('
        print 'Error. Exiting.'
        exit(1)
    
    port = ports[0]
    if verbose:
        print '  Try to connect on ', port
    ctrl = DynamixelController(port, connection_type, timeout = 0.05)
    if verbose:
        print '  Connexion established :', ctrl.io
    
    if motor_range is None:
        motor_range = 0, 253
    if verbose:
        print '  Scanning motors ids between {} and {}'.format(motor_range[0], motor_range[-1])
    
    motors = ctrl.discover_motors(range(motor_range[0],motor_range[-1]+1))
    if len(motors) == 0:
        print '  No motors found.'
        print 'Error. Exiting.'
        exit(0)
    
    ctrl.start_sync()
    
    if verbose:
        print '  {} Motors found : {}'.format(len(motors), motors)
        
    return ctrl
