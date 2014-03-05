from __future__ import print_function, division

import platform
import os
import sys
import glob

from .. import color
#from .. import sinterface

from .controller import (DynamixelController,
                         DynamixelControllerFullRam)

from ..ios.serialio import serialio
from ..ios.serialio import serialcom


TIMEOUTS = {'Darwin': 0,
            'Linux': 0
            }
DEFAULT_TIMEOUT = .01

OK   = '  [{} OK {}] '.format(color.green, color.end)
FAIL = '  [{}FAIL{}] '.format(color.red, color.end)
LOAD = '  [{}LOAD{}] '.format(color.iblue, color.end)


def create_controller(device_type = 'USB2Serial',
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

    mcom = serialcom.SerialCom(sio)

    ctrl_class = DynamixelControllerFullRam if full_ram else DynamixelController
    ctrl = ctrl_class(mcom)

    if verbose:
        print(OK + 'Connexion established: {}{}{}'.format(color.cyan, sio, color.end))

    # discovering motors
    motor_ids = ctrl.discover_motors(range(min(motor_range),
                                           max(motor_range) + 1),
                                     verbose = verbose)
    if verbose:
        print(OK + 'Scanning motor ids between {} and {}...   '.format(
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
        print (LOAD + 'Loading EEPROMs and RAMs...', end='\r'),
        sys.stdout.flush()
    ctrl.load_motors(motor_ids)

    if verbose:
        print(OK + 'Loaded EEPROMs and RAMs.' + 3 * ' ')
        print(OK + 'Models: '
              + ', '.join(["%s%s%s" % (color.cyan, m.model, color.end)
                           for m in ctrl.motors]))

    # staring control loop
    if start:
        ctrl.start()

    # if interface:
    #     pydyn.sinterface = sinterface.SInterface(ctrl, ip = interface_ip,
    #                                              port = interface_port)
    #     pydyn.sinterface.start()

    return ctrl



# vrep_mode = False
# def enable_vrep():
#     global vrep_mode
#     vrep_mode = True

# def create_controller(connection_type = "USB2DXL",
#                       motor_range = (0, 253),
#                       serial_id = None,
#                       timeout = 20,
#                       start = True,
#                       baudrate = 1000000,
#                       usb_device = None,
#                       ip = '127.0.0.1',
#                       ipport = 1986,
#                       full_ram = False,
#                       debug = False,
#                       interface = False,
#                       interface_ip = '127.0.0.1',
#                       interface_port = 31415,
#                       verbose = False,
#                       ):
#     """
#     Guess interface, scans it and configure controller.

#     :arg connection_type:
#         name of connection type, see controller for supported connection types
#     :arg iterable motor_range:
#         only motors between min and max are scanned
#     :arg float timeout:
#         timeout (in s), default to OS based value
#     :arg boolean start:
#         whether to start the controller
#     :arg int baudrate:
#         the baudrate of the serial connection. Motor will be detected when
#         they have a matching baudrate.
#     :arg string usb_device:
#         path to the usb device to use
#     :arg boolean full_ram:
#         default False, whether to load a full RAM controller (see controller).
#     :arg boolean debug:
#         default False whether to display debug messages
#     :arg boolean verbose:
#         whether to display progression messages
#     :return: controller outfitted with all motor found.
#     """

#     if timeout is None:
#         timeout = TIMEOUTS.get(get_os_name(), DEFAULT_TIMEOUT)

#     if vrep_mode:
#         if verbose:
#             print('Loading the robot from V-Rep...')
#         assert io.vrep_available is True
#         connection_type = "VREP"
#         io.DynamixelIO = io.DynamixelIOVRep
#         port = ipport
#         if verbose:
#             print(OK + 'Trying port: {}{}:{}{}'.format(color.bold, ip, port, color.end))

#     else:
#         if verbose:
#             print('Loading the robot from serial bus...')
#         assert io.serial_available is True
#         io.DynamixelIO = io.DynamixelIOSerial
#         port = usb_device
#         if port is None:
#                 ports = get_available_ports()
#             if not ports:
#                 if verbose:
#                     print(FAIL + 'No standart port found. If your port has'
#                         ' a funny name, you might want to specify it '
#                         'explicetly.')
#                 print('Error; exiting.')
#                 exit(1)
#             else:
#                 serial_devnum = 0
#                 if io.use_ftd2xx and serial_id is not None:
#                     for devnum, sid in enumerate(ports):
#                         if sid == serial_id:
#                             port = serial_id
#                             serial_devnum = devnum
#                 else:
#                     port = ports[0]
#         if verbose:
#             print(OK + 'Port found: {}{}{}'.format(color.bold, port, color.end))

#     if io.use_ftd2xx:
#         port = serial_devnum

#     # Create the controller
#     ctrl_class = DynamixelControllerFullRam if full_ram else DynamixelController
#     ctrl = ctrl_class(connection_type, port=port, timeout=timeout,
#                       baudrate=baudrate, ip=ip, debug=debug)


#     if verbose:
#         print(OK + 'Connexion established: {}{}'.format(
#                 color.green, color.end, ctrl.io))

#     motor_ids = ctrl.discover_motors(range(min(motor_range),
#                                            max(motor_range) + 1),
#                                      verbose = verbose)
#     if verbose:
#         print(OK + 'Scanning motor ids between {} and {}...   '.format(
#                 min(motor_range), max(motor_range)))

#     if len(motor_ids) == 0:
#         print (FAIL + 'No motor found. Verify connections, power, USB '
#                'dongle state, scan range.')
#         return ctrl
#     else:
#         if verbose:
#             print(OK + '{} motor(s) found: {}'.format(
#                     len(motor_ids),
#                     ', '.join('{}{}{}'.format(color.cyan, m_id, color.end)
#                               for m_id in motor_ids)))

#     if verbose:
#         print (LOAD + 'Loading EEPROMs and RAMs...\r'),
#         sys.stdout.flush()
#     ctrl.load_motors(motor_ids)

#     if verbose:
#         print(OK + 'Loaded EEPROMs and RAMs.' + 3 * ' ')
#         print(OK + 'Models: '
#               + ', '.join(["%s%s%s" % (color.bold, m.model, color.end)
#                            for m in ctrl.motors]))

#     if start:
#         ctrl.start()

#     if interface:
#         pydyn.sinterface = sinterface.SInterface(ctrl, ip = interface_ip,
#                                                  port = interface_port)
#         pydyn.sinterface.start()

#     return ctrl
