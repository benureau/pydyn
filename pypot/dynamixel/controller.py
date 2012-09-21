import threading
import time

import io
import motor

CONTROLLER_TYPE = ("USB2DXL", "USB2AX")


class DynamixelController(threading.Thread):
    """
        #TODO: Matthieu !!!!
    """
    def __init__(self, port, connection_type, timeout = 0.05):
        threading.Thread.__init__(self)
        self.daemon = True

        if connection_type not in CONTROLLER_TYPE:
            raise ValueError('Unknown controller type: %s' % (connection_type))

        self.type = connection_type
        self.io = io.DynamixelIO(port, timeout = 0.05)
        self.motors = []
        self.motormap = {}
        
    def _configure_motor(self, motor):
        # TODO: check EEPROM from a file
        motor.current_position = motor.goal_position = self.io.get_position(motor.id)
        motor._compliant[1] = not self.io.is_torque_enabled(motor.id)

    def discover_motors(self, motor_ids, load_eeprom = True):
        found_ids = self.io.scan(motor_ids)
        for m_id in found_ids:
            eeprom_data = None
            if load_eeprom:
                eeprom_data = self.read_eeprom(m_id)                
            m = motor.DynamixelMotor(m_id, eeprom_data = eeprom_data)
            self.motors.append(m)
            self.motormap[m.id] = m
            #TODO: check for double motors        
        [self._configure_motor(m) for m in self.motors]
        
        return self.motors

    def read_eeprom(self, motor_id):
        return self.io.read(motor_id, 0, 19)
        
        
    def start_sync(self):
        self.start()

    def _set_properties(self, motor):
        assert motor.flag
        # compliance
        flag, desired, real = motor._compliant
        if flag:
            self.io._set_torque_enable(motor.id, not desired)
            motor._compliant[0] = False
            motor._compliant[2] = not self.io.is_torque_enabled(motor.id)

    def run(self):
        while True:
            start = time.time()
            if self.type == 'USB2AX':
                positions = self.io.get_sync_positions([m.id for m in self.motors])
                for i in range(len(positions)):
                    self.motors[i].current_position = positions[i]

            elif self.type == 'USB2DXL':
                for m in self.motors:
                    m.current_position, m.current_speed, m.current_load = self.io.get_position_speed_load(m.id)

                    # if flag, then something needs changing. 
                    if m.flag:
                        self._set_properties(m)
                        m.flag = False
                    
            sync_pos = []
            for m in self.motors:
                if m.compliant is not None and not m.compliant:
                    sync_pos.append((m.id, m.goal_position))
            if len(sync_pos) > 0:
                self.io.set_sync_positions(sync_pos)
            
            end = time.time()
            dt = 0.020 - (end - start)
            if dt > 0:
                time.sleep(dt)
