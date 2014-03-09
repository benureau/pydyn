from __future__ import print_function, division

import sys

from .controller import (DynamixelController,
                         DynamixelControllerFullRam)
from .. import color
from ..ios.serialio import serialio
from ..ios.serialio import serialcom


OK   = '  [{} OK {}] '.format(color.green, color.end)
FAIL = '  [{}FAIL{}] '.format(color.red, color.end)
LOAD = '  [{}LOAD{}] '.format(color.iblue, color.end)

_ctrlcount = 0
_controllers = {}

def disconnect(uid):
    """Disconnect and delete a controller

    :param uid  the index of the controller
    """
    try:
        ctrl = _controllers.pop(uid)
    except KeyError:
        raise KeyError("no controller with index {} was found".format(uid))
    ctrl.close()
    del _controllers[uid]

def controller(uid):
    """Return the controller with the given uid."""
    try:
        return _controllers[uid]
    except KeyError:
        raise KeyError("no controller with index {} was found".format(uid))

def motors(uid):
    return controller(uid).motors

def connect(device_type = 'USB2Serial',
            serial_id   = None,
            port_path   = None,
            baudrate    = 1000000,
            timeout     = 50,
            full_ram    = False,
            verbose     = True,
            motor_range = (0, 253),
            enable_pyftdi = True,
            start       = True
           ):
    """
    Guess interface, scans it and configure controller.

    :arg device_type:  kind of device for sending messages to the motors.
                       can be either of 'USB2Dynamixel', 'USB2Serial',
                       'USB2AX', 'CM-5', 'CM-510', 'CM-513', 'CM-700', 'CM-900',
                       'OpenCM9.04', 'CM-100', 'CM-100A', 'SerialPort'.
    :arg serial_id:    if multiple usb/serial adaptaters are present, this parameter
                       can connect to a specific one.
    :arg port_path:    an explicit /dev/ path to a device.

        name of connection type, see controller for supported connection types
    """
    global _ctrlcount
    # establishing connection
    if verbose:
        print('Connecting to the serial controller...')

    try:
        sio = serialio.Serial(device_type=device_type,
                                port_path=port_path,
                                serial_id=serial_id,
                                 baudrate=baudrate,
                                  timeout=timeout,
                            enable_pyftdi=enable_pyftdi)
    except serialio.PortNotFoundError as e:
        if verbose:
            print(FAIL + 'No standart port found. If your port has a funny '
                  'name, you might want to specify it explicetly.') # TODO: print device_type, serial_id, port
        print('Error; exiting.')
        exit(1)

    if verbose:
        print(OK + 'Connexion established: {}{}{}'.format(color.cyan, sio, color.end))

    mcom = serialcom.SerialCom(sio)

    ctrl_class = DynamixelControllerFullRam if full_ram else DynamixelController
    ctrl = ctrl_class(mcom)

    _controllers[_ctrlcount] = ctrl
    uid = _ctrlcount
    _ctrlcount += 1

    # discovering motors
    motor_ids = ctrl.discover_motors(range(min(motor_range),
                                           max(motor_range) + 1),
                                     verbose = verbose)
    if verbose:
        print('{}Scanning motor ids between {} and {}...   '.format(OK,
                min(motor_range), max(motor_range)))

    if len(motor_ids) == 0:
        print ('{}No motor found. Verify connections, power, USB '
               'dongle state, scan range.'.format(FAIL))
        return ctrl
    else:
        if verbose:
            print(OK + '{} motor(s) found: {}'.format(
                    len(motor_ids),
                    ', '.join('{}{}{}'.format(color.cyan, m_id, color.end)
                              for m_id in motor_ids)))

    if verbose:
        print ('{}Loading EEPROMs and RAMs...'.format(LOAD), end='\r'),
        sys.stdout.flush()
    ctrl.load_motors(motor_ids)

    if verbose:
        print('{}Loaded EEPROMs and RAMs.   '.format(OK))
        print('{}Models: {}'.format(OK,
                ', '.join(['{}{}{}'.format(color.cyan, m.model, color.end)
                           for m in ctrl.motors])))

    # staring control loop
    if start:
        ctrl.start()

    return uid