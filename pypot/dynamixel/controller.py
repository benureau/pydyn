import threading
import time

import io

CONTROLLER_TYPE = ("USB2DXL", "USB2AX")


class DynamixelController(threading.Thread):
    """
        #TODO: Matthieu !!!!
        """
    def __init__(self, port, connection_type, motors=[]):
        threading.Thread.__init__(self)
        self.daemon = True

        if connection_type not in CONTROLLER_TYPE:
            raise ValueError('Unknown controller type: %s' % (connection_type))

        self.type = connection_type

        self.io = io.DynamixelIO(port)
        self.motors = motors

        [self._configure_motor(m) for m in self.motors]

    def _configure_motor(self, motor):
        # TODO: check EEPROM from a file

        pos = self.io.get_position(motor.id)
        motor.current_position = motor.goal_position = pos


    def start_sync(self):
        self.start()


    def run(self):
        while True:
            start = time.time()
            if self.type == 'USB2AX':
                positions = self.io.get_sync_positions([m.id for m in self.motors])
                for i in range(len(positions)):
                    self.motors[i].current_position = positions[i]

            elif self.type == 'USB2DXL':
                for m in self.motors:
                    m.current_position = self.io.get_position(m.id)

            self.io.set_sync_positions(map(lambda m: (m.id, m.goal_position), self.motors))
            end = time.time()

            dt = 0.020 - (end - start)
            if dt > 0:
                time.sleep(dt)
