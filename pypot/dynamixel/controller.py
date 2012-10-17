import threading
import time
import sys
from collections import OrderedDict
import copy

from .. import color

import io
import motor
import memory

CONTROLLER_TYPE = ("USB2DXL", "USB2AX")


class DynamixelController(threading.Thread):
    """
        The Controller is in charge of handling the read and write request of the motors.
        It is not a data conduit, it merely call io function that update the value of the
        motor cached memory and of the hardware.

        The Controller is also responsible for instanciating Motor instances.

        It does not access the content of the motors memory.
    """

    def __init__(self, port, connection_type, timeout = 0.05, freq = 10):
        """
            :param freq  the target frequence for refreshing values in Hz.
        """

        threading.Thread.__init__(self)
        self.daemon = True

        self.freq = freq


        if connection_type not in CONTROLLER_TYPE:
            raise ValueError('Unknown controller type: %s' % (connection_type))

        self.type = connection_type
        self.io = io.DynamixelIO(port, timeout = timeout)
        self.motors = []

        self._pinglock = threading.Lock() # when discovering motors
        self._ctrllock = threading.Lock() # when running as usual


    # freq and period property, to ensure they remain coherent

    @property
    def freq(self):
        return self._freq

    @freq.setter
    def freq(self, val):
        self._freq = val
        self._period = 1.0/val

    @property
    def period(self):
        return self._period

    @period.setter
    def period(self, val):
        self._freq = 1.0/val
        self._period = val


    # MARK Motor discovery and creation

    def discover_motors(self, motor_ids, load_eeprom = True, verbose = False):
        self._pinglock.acquire()
        self._ctrllock.acquire()

        found_ids = []
        for m_id in motor_ids:
            if verbose:
                print '  [%sSCAN%s] Scanning motor ids between %s and %s : %s\r' % (color.iblue, color.end,
                        motor_ids[0], motor_ids[-1], m_id),
                sys.stdout.flush()
            if self.io.ping(m_id):
                found_ids.append(m_id)

        self._pinglock.release()
        self._ctrllock.release()

        return found_ids

    def load_motors(self, motor_ids):
        #TODO: check for double motors
        return [self.create_motor(motor_id) for motor_id in motor_ids]

    motorclass = {
        'AX' : motor.AXMotor,
        'RX' : motor.RXMotor,
        'MX' : motor.MXMotor,
    }

    def create_motor(self, motor_id):
        """Create a motor instance and load it with EEPROM and RAM data."""
        raw_eeprom = self.read_eeprom(motor_id)
        raw_ram    = self.read_ram(motor_id)
        mmem = memory.DynamixelMemory(raw_eeprom, raw_ram)

        self.io.motormems[mmem.id] = mmem

        m = DynamixelController.motorclass[mmem.modelclass](mmem)

        self.motors.append(m)

        return m

    def read_eeprom(self, motor_id):
        return self.io.read(motor_id, 0, 24 )

    def read_ram(self, motor_id):
        return self.io.read(motor_id, 24, 26)


    # MARK Handling Requests and Updating

    def _reading_present_posspeedload(self):

        if self.type == 'USB2AX':
            positions = self.io.get_sync_positions([m.id for m in self.motors])
            for i in range(len(positions)):
                self.motors[i].current_position = positions[i]

        elif self.type == 'USB2DXL':
            for m in self.motors:
                try:
                    try:
                        position, speed, load = self.io.get_position_speed_load(m.id)
                        m._current_position, m._current_speed, m._current_load = position, speed, load
                    except ValueError as ve:
                        print 'warning: reading status of motor {} failed with : {}'.format(m.id, ve.args[0])

                except io.DynamixelCommunicationError as e:
                    print e
                    print 'warning: communication error on motor {}'.format(m.id)


    pst_set = set(('GOAL_POSITION', 'MOVING_SPEED', 'TORQUE_LIMIT'))

    def _divide_requests(self):
        """This function distributes requests into relevant groups.
            Read request on contiguous variables can be bundled into one packet.

            Ideally, this function should treat all motor request in parallel.
            For the moment, it is limited to one motor at a time and only separate write
            requests on goal_position, moving_speed and torque_limit from the rest.

            :return  list of dictionary request
            """
        # Dividing requests
        all_pst_requests   = []
        all_other_requests = []

        for motor in self.motors:

            motor.request_lock.acquire()
            requests = copy.copy(motor.requests)
            motor.requests.clear()
            motor.request_lock.release()

            pst_requests = OrderedDict()
            other_requests = OrderedDict()

            for request_name, value in requests.items():
                if request_name in DynamixelController.pst_set and value is not None:
                    pst_requests[request_name] = value
                else:
                    other_requests[request_name] = value

            all_pst_requests.append(pst_requests)
            all_other_requests.append(other_requests)

        # copying the resquests (for thread safety)

        return all_pst_requests, all_other_requests

    def _handle_all_pst_requests(self, all_pst_requests):
        # Handling pst requests (if need be)
        sync_pst = []
        for m, pst_requests in zip(self.motors, all_pst_requests):
            if not m.compliant and len(pst_requests) > 0:
                sync_pst.append((m.id,
                                 pst_requests.get('GOAL_POSITION', m.goal_position_raw),
                                 pst_requests.get('MOVING_SPEED', m.moving_speed_raw),
                                 pst_requests.get('TORQUE_LIMIT', m.torque_limit_raw)))

        if len(sync_pst) > 0:
            self.io.set_sync_positions_speeds_torque_limits(sync_pst)

    def _handle_other_requests(self, motor_id, requests):

        # handling the resquests
        for request_name, value in requests.items():
            if value is None:
                self.io.get(motor_id, request_name)
            else:
                self.io.set(motor_id, request_name, value)


    def run(self):
        while True:

            self._pinglock.acquire()
            self._ctrllock.acquire()
            self._pinglock.release()

            start = time.time()

            # reading present position, present speed, present load


            # Dividing requests
            all_pst_requests, all_other_requests = self._divide_requests()

            # Handling pst requests
            self._handle_all_pst_requests(all_pst_requests)

            # Handling other requests
            for m, other_requests in zip(self.motors, all_other_requests):
                self._handle_other_requests(m.id, other_requests)

            self._ctrllock.release()

            end = time.time()
            dt = self._period - (end - start)
            if dt > 0:
                time.sleep(dt)

