import protocol

class DynamixelUnsupportedMotorError(Exception):
    def __init__(self, motor_id, model_number):
        self.motor_id = motor_id
        self.model_number = model_number

    def __str__(self):
        return 'Unsupported Motor with id: %d and model number: %d' % (self.motor_id, self.model_number)

class Memory(object):
    """ 
        This class keeps the last known memory values of a motor.
        
        This class will be used by the Motor class as a read-only container, 
        and by the IO class to retrieve quickly data such as model, status
        return level, wheel/joint mode and adjust its behavior accordingly.
        
        The IO class is the the only class to have write access to the memory
        data and will updateits value each time new data is received from the
        motor through the serial port.
        
        Most notably, the Controller class does not have access to the 
        instances of this class. The Motor class will request update to the 
        controller, which will be executed by the IO, but written in the 
        memory and only communication errors will be returned to the
        Controller.
        
        This class deals only with raw, integer data and does not do any 
        checking on the values.
        
        """
        
    def __init__(self, raw_eeprom):
        """
            :param raw_eeprom  raw eeprom data, ie a list of 19 or 24 integers 
                               each between 0 and 255.
                                
            .. note:: AX series documents EEPROM up to the address 23, while 
                      RX and MX are only documented up to address 18.  
            
        """
        self._memory_data = [None]*70
        self._process_raw_eeprom(raw_eeprom)   

        self.update()
        
    def update(self):
        """Update precalculated values"""
        self.firmware   = self._memory_data[2]
        self.id         = self._memory_data[3]

        try:
            self.model      = protocol.DXL_MODEL_NUMBER[self._memory_data[0]]
            self.modelclass = self.model[:2]
        except KeyError:
            raise DynamixelUnsupportedMotorError(motor_id, model_number)        

        mode_test = (self._memory_data[protocol.DXL_CW_ANGLE_LIMIT] == 
                     self._memory_data[protocol.DXL_CCW_ANGLE_LIMIT] == 0)
        self.mode       = 'wheel' if mode_test else 'joint'

        
    def _process_raw_eeprom(self, rep):
        """Return the eeprom data, with two bytes data properly computed"""
        assert len(rep) >= 19
        self._memory_data[0]  = rep[0] + (rep[1] << 8)
        self._memory_data[2]  = rep[2]
        self._memory_data[3]  = rep[3]
        self._memory_data[3]  = rep[4]
        self._memory_data[5]  = rep[5]
        self._memory_data[6]  = rep[6] + (rep[7] << 8)
        self._memory_data[8]  = rep[8] + (rep[9] << 8)
        self._memory_data[10] = rep[10]  # undocumented
        self._memory_data[11] = rep[11]
        self._memory_data[12] = rep[12]
        self._memory_data[13] = rep[13]
        self._memory_data[14] = rep[14] + (rep[15] << 8)
        self._memory_data[16] = rep[16]
        self._memory_data[17] = rep[17]
        self._memory_data[18] = rep[18]

        if len(rep) >= 24:
            self._memory_data[19] = rep[19]  # undocumented
            self._memory_data[20] = rep[20] + (rep[21] << 8)
            self._memory_data[22] = rep[22] + (rep[23] << 8)
        
    def cache_ram(self, raw_ram):
        """
            RAM values are volatiles, they are reset each time the power is cut.
            Some RAM value can't change unless the user write to them. As such,
            any read request can use the cached values without triggering a 
            serial communication.
            
            :param raw_ram  raw eeprom data, ie a list of 26 or 28 integers 
                            each between 0 and 255.
                            
            .. note:: While AX and RX series documents RAM up to the address 49, 
                      MX series also documents adress 68 and 69 (as two-byte 
                      consuming current value). If the ram list of value is 28, 
                      the last two values are expected to be the one at adress 
                      68 and 69.
        """
        self._process_raw_ram(raw_ram)
           
    def _process_raw_ram(self, rram):
    
        """Return the ram data, with two bytes data properly computed"""
        assert len(rram) >= 26
        self._memory_data[24+ 0] = rram[0],
        self._memory_data[24+ 1] = rram[1],
        self._memory_data[24+ 2] = rram[2],
        self._memory_data[24+ 3] = rram[3],
        self._memory_data[24+ 4] = rram[4],
        self._memory_data[24+ 5] = rram[5],
        self._memory_data[24+ 6] = rram[6]  + (rram[7]  << 8)
        self._memory_data[24+ 8] = rram[8]  + (rram[9]  << 8)
        self._memory_data[24+10] = rram[10] + (rram[11] << 8)
        self._memory_data[24+12] = rram[12] + (rram[13] << 8)
        self._memory_data[24+14] = rram[14] + (rram[15] << 8)
        self._memory_data[24+16] = rram[16] + (rram[17] << 8)
        self._memory_data[24+18] = rram[18],
        self._memory_data[24+19] = rram[19],
        self._memory_data[24+20] = rram[20],
        self._memory_data[24+21] = rram[21], # undocumented
        self._memory_data[24+22] = rram[22], 
        self._memory_data[24+23] = rram[23],
        self._memory_data[24+24] = rram[24] + (rram[25] << 8)

        if len(rram) >= 28:
            self._memory_data[24+26] = rram[26] + (rram[27] << 8)
        
    def __getitem__(self, index):
        return self._memory_data[index]
        
    def __setitem__(self, index, val):
        self.__memory_data[index] = int(val)
        
    def long_desc(self):
        s = '\n'.join('{:2i}: {:4i}'.format(address, value) 
                      for address, value in enumerate(self._memory_data) 
                      if value is not None)
        return s + '\n'