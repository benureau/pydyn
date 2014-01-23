import threading
import time
import sys
from collections import OrderedDict, deque
import copy
import atexit

from .. import color

from . import io
from . import motor
from . import memory
from . import protocol


CONTROLLER_TYPE = ("USB2DXL", "USB2AX", "VREP")

class DynamixelController(threading.Thread):
    """
        The Controller is in charge of handling the read and write request of the motors.
        It is not a data conduit, it merely call io function that update the value of the
        motor cached memory and of the hardware.

        The Controller is also responsible for instanciating Motor instances.

        It does not access the content of the motors memory.
    """

    def __init__(self, connection_type, port=None, timeout=50, freq=50, baudrate=1000000, ip='127.0.0.1', debug=False):
        """
            :arg port:     when using serial, the device port of the connection.
                             when using V-REP, the ip port of the connection.
            :arg freq:     the target frequence for refreshing values in Hz.
            :arg timeout:  the time to wait for a response from the serial port.
            :arg baudrate: the baudrate of the serial connection. Motor will be detected when they have a matching baudrate.
            :arg ip:       when using V-REP, the ip to connect to the V-Rep simulation.
            :arg debug:    if True, displays debug info on the controller behavior.
        """

        threading.Thread.__init__(self)
        self.daemon = True

        self.debug = debug
        self.freq = freq
        self.fps_history = deque(maxlen = 3*freq)
        self.framecount = 0

        if connection_type not in CONTROLLER_TYPE:
            raise ValueError('Unknown controller type: %s' % (connection_type))

        self.type = connection_type
        self.io = io.DynamixelIO(port=port, timeo=timeout, baudrate=baudrate, ip=ip)
        self.motors = []

        def stop_and_close():
            self.stop()
            try:
                self.join(0.5)
            except RuntimeError:
                pass
            self.io.close()
        atexit.register(stop_and_close)

        # atexit.register(self.join, 0.5)
        # atexit.register(self.stop)

        self._pinglock = threading.Lock() # when discovering motors
        self._ctrllock = threading.Lock() # when running as usual

        self._stop = threading.Event()

    def stop(self):
        """Stop the thread after the end of the current loop"""
        self._stop.set()

    def stopped(self):
        return self._stop.isSet()

    def wait(self, loops):
        """ Wait a number of loops. Useful to wait before a change is applied to the motors."""
        if self.is_alive():
            frame = self.framecount
            while(frame + loops >= self.framecount):
                time.sleep(0.001)

    def close(self, immediately=True):
        """ Close the serial connection

            :arg immediately:  if False, wait two additional loops to purge the last orders
        """
        try:
            self.stop()
            self._ctrllock.acquire()
            self.io.close()
            self._ctrllock.release()
        except Exception:
            pass

    # freq and period property, to ensure they remain coherent

    @property
    def freq(self):
        """ Target frequence of the loops. """
        return self._freq

    @freq.setter
    def freq(self, val):
        self._freq = val
        self._period = 1.0/val

    @property
    def period(self):
        """ Same as freq, but inverse. """
        return self._period

    @period.setter
    def period(self, val):
        self._freq = 1.0/val
        self._period = val

    # pausing resuming the controller

    def pause(self):
        """ Pause the looping """
        self._ctrllock.acquire()
        self.stop_sim()

    def resume(self):
        """ Resume the looping after a pause. """
        self.start_sim()
        self._ctrllock.release()

    def restart(self):
        """ Pause and resume the looping """
        self._ctrllock.acquire()
        self.restart_sim()
        self._ctrllock.release()

    def start_sim(self):
        if hasattr(self.io, 'sim'):
            self.io.sim.simStartSimulation()

    def restart_sim(self):
        if hasattr(self.io, 'sim'):
            self.io.sim.simStopSimulation()
            self.io.sim.simStartSimulation()

    def stop_sim(self):
        if hasattr(self.io, 'sim'):
            self.io.sim.simStopSimulation()

    # MARK Motor discovery and creation

    def discover_motors(self, motor_ids, load_eeprom = True, verbose = False):
        self._pinglock.acquire()
        self._ctrllock.acquire()

        found_ids = []
        try:
            found_ids = self.io.broadcast_ping()
        except (AssertionError, AttributeError, IOError):
            for m_id in motor_ids:
                if verbose:
                    print('  [%sSCAN%s] Scanning motor ids between %s and %s : %s\r' % (color.iblue, color.end,
                            motor_ids[0], motor_ids[-1], m_id)),
                    sys.stdout.flush()
                if self.io.ping(m_id):
                    found_ids.append(m_id)

        self._pinglock.release()
        self._ctrllock.release()

        return found_ids

    def load_motors(self, motor_ids):
        #TODO: check for double motors
        mmems = self.io.create(motor_ids)

        for mmem in mmems:
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

    def _reading_motors(self):
        """ Read the motors for position, speed and load """

        if self.type == 'USB2DXL':
            for m in self.motors:
                try:
                    try:
                        self.io.get(m.id, 'PRESENT_POS_SPEED_LOAD')
                    except ValueError as ve:
                        print('warning: reading status of motor {} failed with : {}'.format(m.id, ve.args[0]))

                except io.DynamixelCommunicationError as e:
                    print(e)
                    print('warning: communication error on motor {}'.format(m.id))

        elif self.type == 'USB2AX' or self.type == 'VREP':
            positions = self.io.get_sync_positions([m.id for m in self.motors])




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
        all_read_requests    = []

        for motor in self.motors:

            motor.request_lock.acquire()
            read_requests  = copy.copy(motor.read_requests)
            write_requests = copy.copy(motor.write_requests)
            motor.read_requests.clear()
            motor.write_requests.clear()
            motor.request_lock.release()

            pst_requests     = OrderedDict()
            special_requests = OrderedDict()
            other_requests   = OrderedDict()
            read_requests    = OrderedDict()

            for request_name, value in write_requests.items():
                if request_name in DynamixelController.pst_set:
                    pst_requests[request_name] = value
                elif request_name in DynamixelController.special_set:
                    special_requests[request_name] = value
                else:
                    other_requests[request_name] = value

            all_other_requests.append(other_requests)
            all_pst_requests.append(pst_requests)
            all_special_requests.append(special_requests)
            all_read_requests.append(read_requests)

        # copying the resquests (for thread safety)

        return all_pst_requests, all_special_requests, all_other_requests, all_read_requests

    def _handle_all_pst_requests(self, all_pst_requests):
        # Handling pst requests (if need be)
        sync_pst = []
        sync_st  = []
        for m, pst_requests in zip(self.motors, all_pst_requests):

            if len(pst_requests) > 0:
                if self.debug:
                    print('controller: pst_request:', pst_requests)

                if not m.compliant:
                    sync_pst.append((m.id,
                                     pst_requests.get('GOAL_POSITION', m.goal_position_raw),
                                     pst_requests.get('MOVING_SPEED', m.moving_speed_raw),
                                     pst_requests.get('TORQUE_LIMIT', m.torque_limit_raw)))

                if m.compliant and m.mode == 'joint':
                    if 'MOVING_SPEED' in pst_requests or 'TORQUE_LIMIT' in pst_requests:
                        sync_st.append((m.id, pst_requests.get('MOVING_SPEED', m.moving_speed_raw),
                                              pst_requests.get('TORQUE_LIMIT', m.torque_limit_raw)))


        if len(sync_pst) > 0:
            self.io.set_sync_positions_speeds_torque_limits(sync_pst)

        if len(sync_st) > 0:
            self.io.set_sync_speeds_torque_limits(sync_st)


    def _handle_special_requests(self, all_special_requests):
        # handling the resquests
        for motor, requests in zip(self.motors, all_special_requests):
            for request_name, value in requests.items():
                if request_name == 'ID':
                    self.io.change_id(motor.id, value)
                elif request_name == 'MODE':
                    self.io.change_mode(motor.id, value)
                else:
                    print('REQUEST_NAME', value)
                    raise NotImplementedError


    def _handle_all_other_requests(self, all_other_requests):
        # handling the resquests
        for m, requests in zip(self.motors, all_other_requests):
            for request_name, value in requests.items():
                self.io.set(m.id, request_name, value)

    def _handle_all_read_requests(self, all_read_requests):
        # handling the resquests
        for m, requests in zip(self.motors, all_read_requests):
            for request_name, value in requests.items():
                self.io.get(m.id, request_name)


    def run(self):
        while not self.stopped():

            self.framecount += 1


            self._pinglock.acquire()
            self._ctrllock.acquire()
            self._pinglock.release()

            start = time.time()

            try:
                # reading present position, present speed, present load
                self._reading_motors()

                # Dividing requests
                all_pst_requests, all_special_requests, all_other_requests, all_read_requests = self._divide_requests()

                # Handling requests
                self._handle_all_other_requests(all_other_requests)
                self._handle_all_pst_requests(all_pst_requests)
                self._handle_special_requests(all_special_requests)
                self._handle_all_read_requests(all_special_requests)
            except io.DynamixelCommunicationError:
                print("error ignored")

            self._ctrllock.release()
            time.sleep(0.0001) # timeout to allow lock acquiring by other party

            end = time.time()

            self.fps_history.append(end)
            dt = self._period - (end - start)
            if dt > 0:
                time.sleep(dt)

        self.close()

    @property
    def fps(self):
        """ Number of loop per second over the last 2 seconds """
        len_fps = len(self.fps_history)
        if len_fps < 2:
            return 0.0
        else:
            return len_fps/(self.fps_history[len_fps-1] - self.fps_history[1])


class DynamixelControllerFullRam(DynamixelController):
    """ Controller that reads the entire ram of the motor at every step """

    def _reading_motors(self):
        """ Read the motors entire RAM

        Note that this make USB2AX controllers sync read capability useless.
        """

        for m in self.motors:
            try:
                try:
                    self.io.read_ram(m.id)
                except ValueError as ve:
                    print('warning: reading status of motor {} failed with : {}'.format(m.id, ve.args[0]))

            except io.DynamixelCommunicationError as e:
                print(e)
                print('warning: communication error on motor {}'.format(m.id))

    def _handle_all_read_requests(self, all_read_requests):
        """ Requests for reading ram are ignored in this class, since ram is continously updated. """
        for m, requests in zip(self.motors, all_read_requests):
            for request_name, value in requests.items():
                if not protocol.REG_RAM(request_name):
                    self.io.get(m.id, request_name)
