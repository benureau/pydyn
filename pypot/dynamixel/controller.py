import threading
import time
import sys
from collections import OrderedDict, deque
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

    def __init__(self, port, connection_type, timeout = 0.05, freq = 50, baudrate = 1000000, ip = '127.0.0.1'):
        """
            :param freq  the target frequence for refreshing values in Hz.
        """

        threading.Thread.__init__(self)
        self.daemon = True

        self.freq = freq
        self.fps_history = deque(maxlen = 3*freq)
        self.framecount = 0

        if connection_type not in CONTROLLER_TYPE:
            raise ValueError('Unknown controller type: %s' % (connection_type))

        self.type = connection_type
        self.io = io.DynamixelIO(port, timeout = timeout, baudrate = baudrate, ip = ip)
        self.motors = []

        self._pinglock = threading.Lock() # when discovering motors
        self._ctrllock = threading.Lock() # when running as usual

    def wait(self, loops):
        if self.is_alive():
            frame = self.framecount
            while(frame + loops >= self.framecount):
                time.sleep(0.001)

    def close(self, immediately = False):
        if not immediately:
            self.wait(3)
        self.io.close()

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

    # pausing resuming the controller

    def pause(self):
        self._ctrllock.acquire()
        if hasattr(self.io, 'sim'):
            self.io.sim.simPauseSimulation()

    def resume(self):
        if hasattr(self.io, 'sim'):
            self.io.sim.simStartSimulation()
        self._ctrllock.release()

    def restart(self):
        if hasattr(self.io, 'sim'):
            self.io.sim.simStopSimulation()
            self.io.sim.simStartSimulation()
        self._ctrllock.release()

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
        for motor_id in motor_ids:
            mmem = self.io.create(motor_id)
            m = DynamixelController.motormodel[mmem.model](mmem)
            self.motors.append(m)


    motormodel = {
        'AX-12'   : motor.AXMotor,
        'AX-18'   : motor.AXMotor,
        'AX-12W'  : motor.AXMotor,

        'RX-10'   : motor.RXMotor,
        'RX-24F'  : motor.RXMotor,
        'RX-28'   : motor.RXMotor,
        'RX-64'   : motor.RXMotor,

        'MX-28'   : motor.MXMotor,
        'MX-64'   : motor.MX64Motor,
        'MX-106'  : motor.MX106Motor,

        'EX-106+' : motor.EXMotor,

        'VX-28'   : motor.VXMotor,
        'VX-64'   : motor.VXMotor,
    }

    # MARK Handling Requests and Updating

    def _reading_present_posspeedload(self):

        if self.type == 'USB2AX':
            positions = self.io.get_sync_positions([m.id for m in self.motors])

        elif self.type == 'USB2DXL':
            for m in self.motors:
                try:
                    try:
                        self.io.get(m.id, 'PRESENT_POS_SPEED_LOAD')
                    except ValueError as ve:
                        print 'warning: reading status of motor {} failed with : {}'.format(m.id, ve.args[0])

                except io.DynamixelCommunicationError as e:
                    print e
                    print 'warning: communication error on motor {}'.format(m.id)


    pst_set     = set(('GOAL_POSITION', 'MOVING_SPEED', 'TORQUE_LIMIT'))
    special_set = set(('ID', 'MODE'))


    def _divide_requests(self):
        """This function distributes requests into relevant groups.
            Read request on contiguous variables can be bundled into one packet.

            Ideally, this function should treat all motor request in parallel.
            For the moment, it is limited to one motor at a time and only separate write
            requests on goal_position, moving_speed and torque_limit from special requrest,
            from the rest.

            :return:  list of dictionary request
            """
        # Dividing requests
        all_pst_requests     = []
        all_special_requests = []
        all_other_requests   = []

        for motor in self.motors:

            motor.request_lock.acquire()
            requests = copy.copy(motor.requests)
            motor.requests.clear()
            motor.request_lock.release()

            pst_requests     = OrderedDict()
            special_requests = OrderedDict()
            other_requests   = OrderedDict()

            for request_name, value in requests.items():
                if request_name in DynamixelController.pst_set and value is not None:
                    pst_requests[request_name] = value
                elif request_name in DynamixelController.special_set:
                    special_requests[request_name] = value
                else:
                    other_requests[request_name] = value

            all_other_requests.append(other_requests)
            all_pst_requests.append(pst_requests)
            all_special_requests.append(special_requests)

        # copying the resquests (for thread safety)

        return all_pst_requests, all_special_requests, all_other_requests

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

    def _handle_special_requests(self, all_special_requests):
        # handling the resquests
        for motor, requests in zip(self.motors, all_special_requests):
            for request_name, value in requests.items():
                if request_name == 'ID':
                    if value is None:
                        self.io.get(motor.id, 'ID')
                    else:
                        self.io.change_id(motor.id, value)
                elif request_name == 'MODE':
                    if value is None:
                        self.io.get(motor.id, 'ANGLE_LIMITS')
                    else:
                        self.io.change_mode(motor.id, value)
                else:
                    print 'REQUEST_NAME', value
                    raise NotImplementedError


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
            self._reading_present_posspeedload()

            # Dividing requests
            all_pst_requests, all_special_requests, all_other_requests = self._divide_requests()


            # Handling other requests
            for m, other_requests in zip(self.motors, all_other_requests):
                self._handle_other_requests(m.id, other_requests)

            # Handling pst requests
            self._handle_all_pst_requests(all_pst_requests)

            # Handling special requests
            self._handle_special_requests(all_special_requests)

            self._ctrllock.release()
            time.sleep(0.0001) # timeout to allow lock acquiring by other party

            end = time.time()

            self.framecount += 1
            self.fps_history.append(end)
            dt = self._period - (end - start)
            if dt > 0:
                time.sleep(dt)

    @property
    def fps(self):
        """FPS is computed over the last 2 seconds"""
        len_fps = len(self.fps_history)
        if len_fps < 2:
            return 0.0
        else:
            return len_fps/(self.fps_history[len_fps-1] - self.fps_history[1])

