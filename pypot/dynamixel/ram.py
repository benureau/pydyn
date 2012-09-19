# coding: utf-8
import color

class Ram(object):
    """A class that hold cached value of the RAM of a motor.

    Allows to keep snapshot of the RAM values
    """

    @classmethod 
    def from_raw_data(cls, raw_data, eeprom = None, mode = 'joint', timestamp = 0.0):
        assert len(raw_data) == 26
        data = Ram._process(raw_data)
        return cls(data, eeprom = eeprom, mode = mode, timestamp = timestamp)

    def __init__(self, data, eeprom = None, mode = 'joint', timestamp = None):
        self.data      = data
        self.mode      = mode # joint or wheel
        if eeprom is not None: # determine mode from eeprom
            if eeprom.data[6] == eeprom[8] == 0:
                self.mode = 'wheel'
        self.timestamp = timestamp

    @staticmethod
    def overhead_desc():
        return '    torque     led  cwmarg ccwmarg  cwslop ccwslop goalpos    speed  torqlim   curpos curspeed  curload  curvolt  curtemp registrd   moving     lock    punch'

    def __repr__(self):
        return "RAM({})".format(", ".join("%s%i%s:%s%4i%s" % (color.cyan, i, color.cyan, color.red, v, color.end) for i, v in enumerate(self.data) if v is not None))

    def __len__(self):
        return len(self.data)

    @staticmethod
    def _process(rram):

        """Return the ram data, with two bytes data properly computed"""
        ram = (rram[0], # 24
               rram[1],
               rram[2],
               rram[3],
               rram[4],
               rram[5],
               rram[6]  + (rram[7]  << 8), None, # 30
               rram[8]  + (rram[9]  << 8), None,
               rram[10] + (rram[11] << 8), None,
               rram[12] + (rram[13] << 8), None,
               rram[14] + (rram[15] << 8), None,
               rram[16] + (rram[17] << 8), None, # 40
               rram[18],
               rram[19],
               rram[20],
               rram[21], # undocumented
               rram[22], 
               rram[23],
               rram[24] + (rram[25] << 8), None)
        return ram    

    def long_desc(self):
        """This print the RAM values of the motor.

        @param ram
        From the manual (RX28, RX64)
        Idx  Name                  Description
        24   Torque                Enable Torque On/Off
        25   LED                   LED On/Off
        26   CW Compliance Margin
        27   CCW Compliance Margin 
        28   CW Compliance Slope
        29   CCW Compliance Slope
        30   Goal Position(L)      Lowest byte of Goal Position
        31   Goal Position(H)      Highest byte of Goal Position
        32   Moving Speed(L)       Lowest byte of Moving Speed
        33   Moving Speed(H)       Highest byte of Moving Speed
        34   Torque Limit(L)       Lowest byte of Torque Limit
        35   Torque Limit(H)       Highest byte of Torque Limit
        36   Present Position(L)   Lowest byte of Current Position
        37   Present Position(H)   Highest byte of Current Position
        38   Present Speed(L)      Lowest byte of Current Speed
        39   Present Speed(H)      Highest byte of Current Speed
        40   Present Load(L)       Lowest byte of Current Load
        41   Present Load(H)       Highest byte of Current Load
        42   Present Voltage       Current Voltage
        43   Present Temperature   Current Temperature
        44   Registered            Means if Instruction is registered
        46   Moving                Means if there is any movement
        47   Lock                  Locking EEPROM
        48   Punch(L)              Lowest byte of Punch
        49   Punch(H)              Highest byte of Punch
        """

        def column(address, desc):
            s = "%s%2i%s" % (color.cyan, address+24, color.end)
            if desc == '':
                return (s,"%s%4i" % (color.red, self.data[address]))
            else:
                return (s,"%s%4i  %s%s" % (color.red, self.data[address], color.grey, desc))

        def moving_speed_format(val, mode = None):
            if mode == 'joint':
                if val == 0:
                    return "%6.1f rpm (unleashed)" % (val,)
                return "%6.1f rpm" % (val*0.111)
            elif mode == 'wheel':
                direction, torque10 = divmod(val, 1024)
                return "%5.1f%% torque %s" % (torque10*0.1, ('ccw', 'cw')[direction])
            else:
                return ""

        def present_speed_format(val, mode = None):
            if mode == 'joint':
                return "%4i  (%.1f rpm)" % (val, val*0.111)
            elif mode == 'wheel':
                direction, torque10 = divmod(val, 1024)
                return "%4i  (%.1f%% torque %s)" % (val, torque10//1023, ('ccw', 'cw')[direction])
            else:
                return "%4i" % val

        def load_format(val):
            direction, torque10 = divmod(val, 1024)
            return "%4i  (%.1f%% torque %s)" % (val, 100*torque10//1023, ('ccw', 'cw')[direction])

        def torque_format(val):
            return "%4i  (%.1f%% torque)" % (val, val*0.1)

        s = ['RAM', 
             '%s Torque enabled        : %s' % column( 0, '    ON' if self.data[0] == 1 else '   OFF'),
             '%s LED                   : %s' % column( 1, '    ON' if self.data[1] == 1 else '   OFF'),
             '%s  CW Compliance Margin : %s' % column( 2, '%+6.1f º' % (self.data[2]*0.29)),
             '%s CCW Compliance Margin : %s' % column( 3, '%+6.1f º' % (self.data[3]*0.29)),
             '%s  CW Compliance Slope  : %s' % column( 4, ' level %i/7' % (max(i for i in range(8) if self.data[4] >> i == 1))),
             '%s CCW Compliance Slope  : %s' % column( 5, ' level %i/7' % (max(i for i in range(8) if self.data[5] >> i == 1))),
             '%s Goal Position         : %s' % column( 6, '%+6.1f º' % ((self.data[6]-512)*0.29)),
             '%s Moving Speed          : %s' % column( 8, moving_speed_format(self.data[8], mode = self.mode)),
             '%s Torque Limit          : %s' % column(10, '%6.1f %% of max' % (self.data[10]*0.1)),
             '%s Present Position      : %s' % column(12, '%+6.1f º' % ((self.data[12]-512)*0.29)),
             '%s Present Speed         : %s' % column(14, moving_speed_format(self.data[14], mode = self.mode)),
             '%s Present Load          : %s' % column(16, '%6.1f %% torque %s' % (100.0*(self.data[16] % 1024)//1023, ('ccw', 'cw')[self.data[16]/1024])),
             '%s Present Voltage       : %s' % column(18, '%6.1f V' % (self.data[18]*0.1)),
             '%s Present Temperature   : %s' % column(19, '%6i ºC' % (self.data[19])),
             '%s Registered            : %s' % column(20, '    NO' if self.data[20] == 0 else '   YES'),
             '%s Moving                : %s' % column(22, '    NO' if self.data[22] == 0 else '   YES'),
             '%s Lock                  : %s' % column(23, '    NO' if self.data[23] == 0 else '   YES'),
             '%s Punch                 : %s' % column(24, '%6.1f %% torque' % (self.data[24]*0.1)),
             '\n']

        return '\n'.join(s)
