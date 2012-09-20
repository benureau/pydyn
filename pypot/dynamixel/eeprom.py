# coding: utf-8
import color
import conversions

class Eeprom(object):
    """A class that hold cached value of the EEPROM of a motor.

    Allows instantaneous access when eeprom values are requested.
    Implements interface to propagate change to the server.
    """

    @classmethod 
    def from_raw_data(cls, raw_data):
        assert len(raw_data) == 19
        data = Eeprom._process(raw_data)
        return cls(data)

    def __init__(self, data):
        self.data = data

        self.model    = conversions.raw_to_model(self.data[0])
        self.firmware = self.data[2]
        self.id       = self.data[3]

    # MARK  processing data

    @staticmethod
    def _process(rep):
        """Return the eeprom data, with two bytes data properly computed"""
        ep = (rep[0] + (rep[1] << 8), None,
              rep[2],
              rep[3],
              rep[4],
              rep[5],
              rep[6] + (rep[7] << 8), None,
              rep[8] + (rep[9] << 8), None,
              rep[10],  # undocumented
              rep[11],
              rep[12],
              rep[13],
              rep[14] + (rep[15] << 8), None,
              rep[16],
              rep[17],
              rep[18])         
        return ep

    
    def __len__(self):
        return len(self.data)


    # MARK  Printing methods

    def __repr__(self):
        return "EEPROM({})".format(", ".join(
            "%s%i%s:%s%4i%s" % (color.cyan, i, color.cyan, color.red, v, color.end)
            for i, v in enumerate(self.data) if v is not None
            ))

    @staticmethod
    def overhead_desc():
        return '        model    firm      id    baud   delay   cwang  ccwang'\
         '  maxtemp  mintemp  maxvolt  minvolt  maxtorq   status      led shutdown'

    def long_desc(self):
        """This print the EEPROM values in of the motor

        From the manual (RX28, RX64)
        Idx  Name                 Description
        0    Model Number(L)      Lowest byte of model number
        1    Model Number(H)      Highest byte of model number
        2    Firmware             Information on the version of firmware
        3    ID                   ID of Dynamixel
        4    Baud Rate            Baud Rate of Dynamixel
        5    Return Delay Time    Return Delay Time
        6    CW Angle Limit(L)    Lowest byte of clockwise Angle Limit
        7    CW Angle Limit(H)    Highest byte of clockwise Angle Limit
        8    CCW Angle Limit(L)   Lowest byte of counterclockwise Angle Limit
        9    CCW Angle Limit(H)   Highest byte of counterclockwise Angle Limit
        11   Max Limit Temp       Internal Limit Temperature
        12   Min Limit Voltage    Lowest Limit Voltage
        13   Max Limit Voltage    Highest Limit Voltage
        14   Max Torque(L)        Lowest byte of Max. Torque
        15   Max Torque(H)        Highest byte of Max. Torque
        16   Status Return Level  -
        17   Alarm LED            LED for Alarm
        18   Alarm Shutdown       Shutdown for Alarm
        """
        if self.data[0] != 28 and self.data[0] != 64:
            print 'eeprom_desc: dynamixels other than RX-28 or RX-64 are not fully supported'
            return

        def binary(a):
            assert 0 <= a < 256
            s = ''
            for i in range(7, -1, -1):
                q, a = divmod(a, 2**i)
                s += str(q)
            return s

        alarm_shutdown = ('input voltage',
                          'angle limit',
                          'overheating',
                          'range',
                          'checkSum',
                          'overload',
                          'instruction')

        return_status = ('return only ping',
                         'return ping and read commands',
                         'return all commands')

        bled      = binary(self.data[17])
        aled      = bled + ": " + ", ".join(alarm_shutdown[i] for i in range(7) if bled[7-i] == '1')
        bshutdown = binary(self.data[18])
        ashutdown = bshutdown + ": " + ", ".join(alarm_shutdown[i] for i in range(7) if bshutdown[7-i] == '1')
        mode      = 'wheel' if self.data[6] == 0 and self.data[8] == 0 else 'joint'

        def column(address, desc):
            s = "%s%2i%s" % (color.cyan, address, color.end)
            if desc == '':
                return (s,"%s%4i" % (color.red, self.data[address]))
            else:
                return (s,"%s%4i  %s%s" % (color.red, self.data[address], color.grey, desc))


        if self.data[14] == 1023:
            torque_desc = '  100 %'
        else:
            torque_desc = ' %4.1f %%' % (self.data[14]//1023,) 

        s = ['EEPROM', 
             '%s Model                 : %s' % column(0,  'RX-%i' % (self.data[0],)),
             '%s Firware               : %s' % column(2,  ''),
             '%s ID                    : %s' % column(3,  ''),
             '%s Baud Rate             : %s' % column(4,  '%i bauds' % (2000000/(self.data[4]+1))),
             '%s Return Delay Time     : %s' % column(5,  '%i usec' % (2*self.data[5])),
             '%s CW Angle Limit        : %s' % column(6,  '%+.1f º' % ((self.data[6])*0.29)),
             '%s CCW Angle Limit       : %s' % column(8,  '%+.1f º, %s mode' % (((self.data[8])*0.29), mode)),
             '%s Max Limit Temp        : %s' % column(11, ' %.1f C' % (self.data[11],)),
             '%s Min Limit Voltage     : %s' % column(12, ' %4.1f V' % (self.data[12]*0.1,)),
             '%s Max Limit Voltage     : %s' % column(13, ' %4.1f V' % (self.data[13]*0.1,)),
             '%s Max Torque            : %s' % column(14, torque_desc),
             '%s Status Return Level   : %s' % column(16, return_status[self.data[16]]),
             '%s Alarm LED             : %s' % column(17, aled),
             '%s Alarm Shutdown        : %s' % column(18, ashutdown),
             '\n'+color.end]
        return ('\n').join(s)
