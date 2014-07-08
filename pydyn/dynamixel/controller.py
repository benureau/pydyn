from __future__ import print_function, division

import threading
import time
import sys
from collections import OrderedDict, deque
import copy
import atexit

from .. import color

from ..refs import protocol as pt
from . import motor


class DynamixelController(threading.Thread):
    """
    The Controller is in charge of handling the read and write request of the motors.
    It is not a data conduit, it merely call the io function that update the hardware
    and the motor cached memory. It does not access the content of the motors memory.

    The Controller is also responsible for instanciating Motor instances.

    The Controller is not responsible for instanciating the motor communication object,
    it expects a functionnal instance.
    """

    def __init__(self, motorcom, broadcast_ping=False, freq=60, debug=False):
        """
        :arg motorcom:  motor communication object
        :arg freq:      the target frequence for refreshing values in Hz.
        :arg debug:     if True, displays debug info on the controller behavior.
        """

        threading.Thread.__init__(self)
        self.daemon = True

        self.debug = debug
        self.freq = freq
        self.fps_history = deque(maxlen = 3*freq)
        self.framecount = 0
        self.broadcast_ping = broadcast_ping

        self.com = motorcom
        self.motors = []
        self._mtimeouts = {} # motor timeouts after EEPROM writes

        def stop_and_close():
            self.stop()
            try:
                self.join(0.5)
            except RuntimeError:
                pass
            self.com.close()
        atexit.register(stop_and_close)

        self._pinglock = threading.Lock() # when discovering motors
        self._ctrllock = threading.Lock() # when running the control loop

        self._stop_event = threading.Event()

    def stop(self):
        """Stop the thread after the end of the current loop"""
        self._stop_event.set()

    def stopped(self):
        return self._stop_event.is_set()

    def wait(self, loops):
        """ Wait a number of loops. Useful to wait before a change is applied to the motors."""
        if self.is_alive():
            frame = self.framecount
            while frame + loops >= self.framecount:
                time.sleep(0.001)

    def close(self, immediately=True):
        """ Close the serial connection

            :arg immediately:  if False, wait two additional loops to purge the last orders
        """
        try:
            self.stop()
            if not immediately:
                self.join(0.1)
            self._ctrllock.acquire()
            self.com.close()
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
        if hasattr(self.com, 'sim'):
            self.com.sim.simStartSimulation()

    def restart_sim(self):
        if hasattr(self.com, 'sim'):
            self.com.sim.simStopSimulation()
            self.com.sim.simStartSimulation()

    def stop_sim(self):
        if hasattr(self.com, 'sim'):
            self.com.sim.simStopSimulation()

    # MARK Motor discovery and creation

    def discover_motors(self, motor_ids=range(0, 253), verbose = False):

        self._pinglock.acquire()
        self._ctrllock.acquire()

        try:
            found_ids = []
            if self.broadcast_ping:
                try:
                    found_ids = self.com.robust_ping_broadcast()
                except IOError:
                    pass

            if found_ids == []:
                for m_id in motor_ids:
                    if verbose:
                        print('  [{}SCAN{}] Scanning motor ids between {} and {} : {}'.format(color.iblue, color.end,
                                motor_ids[0], motor_ids[-1], m_id), end='\r')
                        sys.stdout.flush()
                    if self.com.ping(m_id):
                        found_ids.append(m_id)

            def in_mids(a):
                return a in motor_ids
            found_ids = [mid for mid in found_ids if mid in motor_ids]

        except self.com.MotorError as e:
            for m in self.motors:
                if m.id == e.mid:
                    m._error.append(e)

        self._pinglock.release()
        self._ctrllock.release()

        return found_ids

    def load_motors(self, motor_ids):
        """Instanciate a set of motors"""
        motor_ids = sorted(set(motor_ids)) # eliminating doublons.
        try:
            mmems = self.com.create(motor_ids)
        except self.com.MotorError as e:
            for m in self.motors:
                if m.id == e.mid:
                    m._error.append(e)

        for mmem in mmems:
            m = motor.MOTOR_MODELS[mmem.model](mmem)
            self.motors.append(m)


    # MARK Handling Requests and Updating

    def _reading_motors(self):
        """ Read the motors for position, speed and load """
        now = time.time()
        mids = [m.id for m in self.motors if self._mtimeouts.get(m.id, now) <= now]
        if len(mids) > 0:
            try:
                self.com.get(pt.PRESENT_POS_SPEED_LOAD, [m.id for m in self.motors if self._mtimeouts.get(m.id, now) <= now])
            except self.com.TimeoutError as e:
                self.com.purge()
                raise e

        # TODO: error policy class
        # if self.com == 'USB2DXL':
        #     for m in self.motors:
        #         try:
        #             try:
        #                 self.com.get(m.id, 'PRESENT_POS_SPEED_LOAD')
        #             except (ValueError, io.DynamixelTimeoutError, io.DynamixelCommunicationError) as ve:
        #                 print('[{}] warning: reading status of motor {} failed with : {}'.format(self.framecount, m.id, ve))
        #                 self.com._serial.purge()
        #                 self.com._serial.resetDevice()
        #                 self.com._serial.setTimeouts(self.com._timeout, self.com._timeout)
        #                 self.com._serial.setLatencyTimer(2)
        #         except io.DynamixelCommunicationError as e:
        #             print(e)
        #             print('warning: communication error on motor {}'.format(m.id))

        # elif self.type == 'USB2AX' or self.type == 'VREP':
        #     positions = self.com.get_sync_positions([m.id for m in self.motors])

    pst_set     = set((pt.GOAL_POSITION, pt.MOVING_SPEED, pt.TORQUE_LIMIT))
    special_set = set((pt.ID))

    def _divide_motor_requests(self, motor):
        """\
        Divide a motor request between read, position/speed/torque_limit writes
        and other writes.
        If an eeprom write is present, only extract this one, and leave the other
        requests on the motor.
        """
        write_rq     = OrderedDict()
        write_pst_rq = OrderedDict()

        now = time.time()
        if self._mtimeouts.get(motor.id, now) > now:
            return write_rq, write_pst_rq, OrderedDict()

        motor.request_lock.acquire()
        for control, value in motor.write_requests.items():
            if not control.ram:
                # we have a eeprom write to do, so we do it exclusively and set
                # a timeout on the motor. Other request will be treated when
                # the timeout clears.
                write_rq[control] = value
                motor.write_requests.pop(control)
                motor.request_lock.release()
                return write_rq, write_pst_rq, OrderedDict()

        # if we're here, we don't have any eeprom write to do
        # all request will be treated.
        write_requests = copy.copy(motor.write_requests)
        motor.write_requests.clear()
        read_rq  = copy.copy(motor.read_requests)
        motor.read_requests.clear()
        motor.request_lock.release()


        for control, value in write_requests.items():
            if control in DynamixelController.pst_set:
                write_pst_rq[control] = value
            else:
                write_rq[control] = value

        return write_rq, write_pst_rq, read_rq

    def _divide_requests(self):
        """This function distributes requests into relevant groups.
            Read request on contiguous variables can be bundled into one packet.

            Ideally, this function should treat all motor request in parallel.
            For the moment, it is limited to one motor at a time and only separate write
            requests on goal_position, moving_speed and torque_limit from special requrest,
            from the rest.

            :return:  list of dictionary request
            """
        write_pst_requests = []
        write_requests     = []
        read_requests      = []


        for motor in self.motors:
            m_write, m_pst, m_read = self._divide_motor_requests(motor)

            write_requests.append(m_write)
            write_pst_requests.append(m_pst)
            read_requests.append(m_read)

        return write_requests, write_pst_requests, read_requests

    def _handle_all_pst_requests(self, all_pst_requests):
        # Handling pst requests (if need be)
        sync_pst = []
        sync_st  = []
        pst_mids = []
        pst_valuess   = []
        st_mids = []
        st_valuess   = []
        for m, pst_reqs in zip(self.motors, all_pst_requests):
            if len(pst_reqs) > 0:

                # if not m.compliant:
                pst_mids.append(m.id)
                pst_valuess.append((pst_reqs.get(pt.GOAL_POSITION, m.goal_position_bytes),
                                    pst_reqs.get(pt.MOVING_SPEED,  m.moving_speed_bytes),
                                    pst_reqs.get(pt.TORQUE_LIMIT,  m.torque_limit_bytes)))

                # elif m.mode == 'joint':
                #     st_mids.append(m.id)
                #     if pt.MOVING_SPEED in pst_reqs or pt.TORQUE_LIMIT in pst_reqs:
                #         st_valuess.append((pst_reqs.get(pt.MOVING_SPEED, m.moving_speed_raw),
                #                           pst_reqs.get(pt.TORQUE_LIMIT, m.torque_limit_raw)))


        if len(pst_valuess) > 0:
            self.com.set(pt.GOAL_POS_SPEED_TORQUE, pst_mids, pst_valuess)

        if len(st_valuess) > 0:
            self.com.set(pt.GOAL_POS_SPEED_TORQUE, st_mids, st_valuess)


    def _handle_special_requests(self, all_special_requests):
        # handling the resquests
        for motor, requests in zip(self.motors, all_special_requests):
            for control, value in requests.items():
                if control == pt.ID:
                    self.com.change_id(motor.id, value)


    def _handle_write_requests(self, write_requests):
        # handling the resquests
        for m, requests in zip(self.motors, write_requests):
            for control, values in requests.items():
                if control == pt.ID:
                    self.com.change_id(m.id, values)
                else:
                    if not hasattr(values, '__iter__'):
                        values = (values,)
                    self.com.set(control, (m.id,), (values,))
                if not control.ram:
                    now = time.time()
                    self._mtimeouts[m.id] = max(self._mtimeouts.get(m.id, now), now)+0.020*len(control.sizes)

    def _handle_all_read_rq(self, all_read_rq):
        # handling the resquests
        for m, requests in zip(self.motors, all_read_rq):
            for control, value in requests.items():
                self.com.get(control, (m.id,))


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
                write_requests, write_pst_requests, read_requests = self._divide_requests()

                # Handling requests
                self._handle_write_requests(write_requests)
                self._handle_all_pst_requests(write_pst_requests)
                self._handle_all_read_rq(read_requests)
            except (self.com.CommunicationError, self.com.TimeoutError) as e:
                for m in self.motors:
                    if m.id == e.mid:
                        m._error.append(e)
                self.stop()

            except self.com.MotorError as e:
                for m in self.motors:
                    if m.id == e.mid:
                        m._error.append(e)
                self.stop()

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
                    self.com.read_ram(m.id)
                except ValueError as ve:
                    print('warning: reading status of motor {} failed with : {}'.format(m.id, ve.args[0]))

            except self.com.DynamixelCommunicationError as e:
                print(e)
                print('warning: communication error on motor {}'.format(m.id))

    def _handle_all_read_rq(self, all_read_rq):
        """ Requests for reading ram are ignored in this class, since ram is continously updated. """
        for m, requests in zip(self.motors, all_read_rq):
            for control in requests.keys():
                if not control.ram:
                    self.com.get(control, m.id)
