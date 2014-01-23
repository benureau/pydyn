import os
import sys
import glob

import ftd2xx

from .. import color
from .. import sinterface

import pydyn.dynamixel.io
from pydyn.dynamixel.controller import (DynamixelController,
                                        DynamixelControllerFullRam)

vrep_mode = False
TIMEOUTS = {'Darwin': 0,
            'Linux': 0
            }
DEFAULT_TIMEOUT = .01

OK = '  [{} OK {}] '.format(color.green, color.end)
FAIL = '  [{}FAIL{}] '.format(color.red, color.end)
LOAD = '  [{}LOAD{}] '.format(color.iblue, color.end)


def enable_vrep():
    global vrep_mode
    vrep_mode = True


def get_os_name():
    return os.uname()[0]


def get_available_ports():
    """
        Returns the available ports found.

        The available ports are looked for in the following path depending on the OS:
            * Linux: /dev/ttyACM* and /dev/ttyUSB*
            * Mac OS: /dev/tty.usb*

       :raises: NotImplementedError if your OS is not one of the currently supported (Linux, Mac OS).

        """
    op_system = get_os_name()

    if op_system == 'Darwin':
        return glob.glob('/dev/cu.usb*') + glob.glob('/dev/tty.usb*')

    elif op_system == 'Linux':
        return glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')

    raise NotImplementedError('Unknown operating system: %s' % (op_system))


def create_controller(connection_type = "USB2DXL",
                      motor_range = (0, 253),
                      timeout = None,
                      start = True,
                      baudrate = 1000000,
                      usb_device = None,
                      ip = '127.0.0.1',
                      ipport = 1986,
                      full_ram = False,
                      debug = False,
                      interface = False,
                      interface_ip = '127.0.0.1',
                      interface_port = 31415,
                      verbose = False,
                      ):
    """Guess interface, scans it and configure controller.

    :arg connection_type:
        name of connection type, see controller for supported connection types
    :arg iterable motor_range:
        only motors between min and max are scanned
    :arg float timeout:
        timeout (in s), default to OS based value
    :arg boolean start:
        whether to start the controller
    :arg int baudrate:
        the baudrate of the serial connection. Motor will be detected when
        they have a matching baudrate.
    :arg string usb_device:
        path to the usb device to use
    :arg boolean full_ram:
        default False, whether to load a full RAM controller (see controller).
    :arg boolean debug:
        default False whether to display debug messages
    :arg boolean verbose:
        whether to display progression messages
    :return: controller outfitted with all motor found.
    """

    if timeout is None:
        timeout = TIMEOUTS.get(get_os_name(), DEFAULT_TIMEOUT)

    if vrep_mode:
        if verbose:
            print('Loading the robot from V-Rep...')
        assert pydyn.dynamixel.io.vrep_available is True
        connection_type = "VREP"
        pydyn.dynamixel.io.DynamixelIO = pydyn.dynamixel.io.DynamixelIOVRep
        port = ipport
        if verbose:
            print(OK + 'Trying port: {}{}:{}{}'.format(color.bold, ip, port, color.end))

    else:
        if verbose:
            print('Loading the robot from serial bus...')
        assert pydyn.dynamixel.io.serial_available is True
        pydyn.dynamixel.io.DynamixelIO = pydyn.dynamixel.io.DynamixelIOSerial
        port = usb_device
        if port is None:
            ports = ftd2xx.listDevices()
            if not ports:
                if verbose:
                    print(FAIL + 'No standart port found. If your port has'
                        ' a funny name, you might want to specify it '
                        'explicetly.')
                print('Error; exiting.')
                exit(1)
            port = ports[0]
        if verbose:
            print(OK + 'Port found: {}{}{}'.format(color.bold, port, color.end))

    # Create the controller
    ctrl_class = DynamixelControllerFullRam if full_ram else DynamixelController
    ctrl = ctrl_class(connection_type, port=port, timeout=timeout,
                      baudrate=baudrate, ip=ip, debug=debug)


    if verbose:
        print(OK + 'Connexion established: {}{}'.format(
                color.green, color.end, ctrl.io))

    motor_ids = ctrl.discover_motors(range(min(motor_range),
                                           max(motor_range) + 1),
                                     verbose = verbose)
    if verbose:
        print(OK + 'Scanning motor ids between {} and {}'.format(
                min(motor_range), max(motor_range)))

    if len(motor_ids) == 0:
        print (FAIL + 'No motor found. Verify connections, power, USB '
               'dongle state, scan range.')
        return ctrl
    else:
        if verbose:
            print(OK + '{} motor(s) found: {}'.format(
                    len(motor_ids),
                    ', '.join('{}{}{}'.format(color.cyan, m_id, color.end)
                              for m_id in motor_ids)))

    if verbose:
        print (LOAD + 'Loading EEPROMs and RAMs...\r'),
        sys.stdout.flush()
    ctrl.load_motors(motor_ids)

    if verbose:
        print(OK + 'Loaded EEPROMs and RAMs.' + 3 * ' ')
        print(OK + 'Models: '
              + ', '.join(["%s%s%s" % (color.bold, m.model, color.end)
                           for m in ctrl.motors]))

    if start:
        ctrl.start()

    if interface:
        pydyn.sinterface = sinterface.SInterface(ctrl, ip = interface_ip,
                                                 port = interface_port)
        pydyn.sinterface.start()

    return ctrl
