import eeprom
import conversions

class DynamixelMotor(object):
    """ """
    def __init__(self, m_id, eeprom_data = None):
        self._id = m_id

        self.eeprom = None
        if eeprom_data is not None:
            self.eeprom = eeprom.Eeprom.from_raw_data(eeprom_data)
        
        self._current_position = None
        self._current_speed    = None
        self._current_load     = None
        
        self.goal_position     = None
        self.moving_speed      = None
        self.torque_limit      = 100.0

        # if flag is True, the controller needs to change other values
        # than the position, speed or load.
        self.flag = False
        
        # Each of those property _p follow the same construction
        #     _p[0] is a flag. If True, the controller needs to update the property on the motor
        #     _p[1] is the desired value
        #     _p[2] is the real value
        # Even if the desired value and the real value don't match, if the flag is not up,
        # the controller will not try to change it. This is used to deal with communication error,
        # and non legal value. All this mechanism is hidden away in properties. 
        self._compliant = [False, None, None]

        self.properties = [self._compliant]

    def __repr__(self):
        return 'M{}'.format(self.id)
         
    # MARK EEPROM properties

    @property
    def id(self):
        if self.eeprom is None:
            return self._id
        else:
            return self.eeprom.id

    @property
    def model(self):
        return self.eeprom.model
        
    @property
    def version(self):
        return self.eeprom.firmware
    
    @property
    def baudrate(self):
        return conversions.raw_to_baudrate(self.eeprom.data[4])
    
    @property
    def return_delay_time(self):
        return conversions.raw_to_return_delay_time(self.eeprom.data[5])
    
    @property
    def cw_angle_limit(self):
        return conversions.position_to_angle(self.eeprom.data[6], self.model)
        
    @property
    def ccw_angle_limit(self):
        return conversions.position_to_angle(self.eeprom.data[8], self.model)
    
    @property
    def max_temp(self):
        return conversions.raw_to_temperature(self.eeprom.data[11])
    
    @property
    def min_voltage(self):
        return conversions.raw_to_voltage(self.eeprom.data[12])
    
    @property
    def max_voltage(self):
        return conversions.raw_to_voltage(self.eeprom.data[13])
    
    @property
    def max_torque(self):
        return conversions.raw_to_torque(self.eeprom.data[14])
    
    @property
    def return_status(self):
        return self.eeprom.data[16]


    # MARK RAM properties

    @property
    def current_position(self):
        return self._current_position

    @property
    def current_speed(self):
        return self._current_speed

    @property
    def current_load(self):
        return self._current_load

    @property
    def position(self):
        return self._current_position

    @position.setter
    def position(self, val):
        self.goal_position = val

    @property
    def speed(self):
        return self._current_speed
        
    @speed.setter
    def speed(self, val):
        self.moving_speed = val
    
    @property 
    def compliant(self):
        return self._compliant[2]
    
    @compliant.setter
    def compliant(self, val):
        self._compliant[1] = val
        if val != self._compliant[2]:
            self._compliant[0] = True
            self.flag = True
