import os
import sys
import glob

from .. import color
from .. import sinterface

import pydyn.dynamixel.io
from pydyn.dynamixel.controller import DynamixelController, DynamixelControllerFullRam

vrep_mode = False


def enable_vrep():
    global vrep_mode
    vrep_mode = True


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


def create_controller(connection_type = "USB2DXL", verbose = False, motor_range = None, timeout = None, start = True, baudrate = 1000000, ip = '127.0.0.1', ipport = 1984, full_ram = False, debug = False, interface = False, interface_ip = '127.0.0.1', interface_port = 1984):
    """ Return a controller outfitted with all motor found. """

    if timeout is None:
        op_system = os.uname()[0]
        if op_system == 'Darwin':
            timeout = 0.02
        elif op_system == 'Linux':
            timeout = 0.005
        else:
            timeout = 0.01

    if vrep_mode:
        if verbose:
            print('Loading the robot from V-Rep...')
        assert pydyn.dynamixel.io.vrep_available is True
        pydyn.dynamixel.io.DynamixelIO = pydyn.dynamixel.io.DynamixelIOVRep
        port = ipport
        if verbose:
            print('  [{} OK {}] Trying port : {}{}:{}{}'.format(color.green, color.end, color.bold, ip, port, color.end))

    else:
        if verbose:
            print('Loading the robot from serial bus...')
        assert pydyn.dynamixel.io.serial_available is True
        pydyn.dynamixel.io.DynamixelIO = pydyn.dynamixel.io.DynamixelIOSerial
        ports = get_available_ports()
        if not ports:
            if verbose:
                print '  [{}FAIL{}] No standart port found. If you port has a funny name, you might want to specify it explicetly.'.format(color.red, color.end)
            print 'Error; exiting.'
            exit(1)

        port = ports[0]
        if verbose:
            print('  [{} OK {}] Port found : {}{}{}'.format(color.green, color.end, color.bold, port, color.end))

    if full_ram:
        ctrl = DynamixelControllerFullRam(port, connection_type, timeout = timeout, baudrate = baudrate, ip = ip, debug = debug)
    else:
        ctrl = DynamixelController(port, connection_type, timeout = timeout, baudrate = baudrate, ip = ip, debug = debug)
    if verbose:
        print '  [{} OK {}] Connexion established : {}'.format(color.green, color.end, ctrl.io)

    if motor_range is None:
        motor_range = 0, 253
    # if verbose:
    #     print '  Scanning motors ids between {} and {}'.format(motor_range[0], motor_range[-1])

    motor_ids = ctrl.discover_motors(range(motor_range[0], motor_range[-1] + 1), verbose = verbose)
    if verbose:
        print '  [{} OK {}] Scanning motor ids between {} and {}             '.format(color.green,
                color.end, motor_range[0], motor_range[-1])

    if len(motor_ids) == 0:
        print '  [{}FAIL{}] No motor found. Verify connections, power, USB dongle state, scan range.'.format(color.red,
        color.end)
        return ctrl
    else:
        if verbose:
            print '  [{} OK {}] Motors found : {}'.format(color.green, color.end, ', '.join('{}{}{}'.format(color.cyan, m_id, color.end) for m_id in motor_ids))

    if verbose:
        print '  [{}LOAD{}] Loading EEPROMs and RAMs...\r'.format(color.iblue, color.end),
        sys.stdout.flush()
    ctrl.load_motors(motor_ids)

    if verbose:
        print '  [{} OK {}] Loaded EEPROMs and RAMs     '.format(color.green, color.end)
        print '  [{} OK {}] Models : {}'.format(color.green, color.end,
                ', '.join(['{}{}{}'.format(color.bold, m.model, color.end) for m in ctrl.motors]))

    if start:
        ctrl.start()

    if interface:
        pydyn.sinterface = sinterface.SInterface(ctrl, ip = interface_ip, port = interface_port)
        pydyn.sinterface.start()

    return ctrl
