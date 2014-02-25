"""
Motor memory. Keeps a cached image of the motor memory.
It is only as uptodate as the last read of each address.
"""

from ..io import protocol as pt
from . import memsave

class DynamixelMemory(object):
    """
    This class keeps the last known memory values of a motor.

    This class will be used by the Motor class as a read-only container,
    and by the Com class to write changes, and access behavioral state
    (model, modelclass, status, return level, wheel/joint mode).

    The Com class is the the only class to have write access to the memory
    data and will update its value each time new data is received from the
    motor through the serial port.

    Most notably, the Controller class does not have access to the
    instances of this class. The Motor class will request updates to the
    controller, which will be executed by the Com, but written here to be
    accessed by the Motor class directly. Com will only transmit
    communication errors to the Controller.

                read            (mostly) write
        Motor <------- Memory <----------------(>) Com
          |                                         ^
          |                                         |
          L--------------> Controller --------------J
             request r/w               request r/w

    This class deals only with raw, integer data and does not do any
    checking on the values.
    """

    def __init__(self, mid, save=False, memory=None):
        """Initialize the memory.

        :param mid:    the id of the motor
        :param save:   if True, all memory changes are sparsely changed.
        :param memory: if given, initialize the memory with that data.
        """
        if memory is not None:
            self._memory_data = memory
        else:
            self._memory_data = [None]*74

        self.save = save
        self.history = memsave.MemSave(74)

        self.id = mid
        self.status_return_level = 1 # to make first read possible
        self.model, self.modelclass = None, None
        self.mode, self.lock = None, None

        if memory is not None:
            self.update()

    # def __init__(self, raw_eeprom, raw_ram, save=False):
    #     """
    #         :param raw_eeprom  raw eeprom data, ie a list of 19 or 24 integers
    #                            each between 0 and 255.
    #         :param raw_ram     raw ram data, ie a list of 24 or 26 integers
    #                            each between 0 and 255.

    #         .. note:: AX series documents EEPROM up to the address 23, while
    #                   RX and MX are only documented up to address 18.

    #     """
    #     self._memory_data = [None]*74
    #     self.save = save
    #     self.history = memsave.MemSave(74)

    #     self._process_raw_eeprom(raw_eeprom)
    #     self._process_raw_ram(raw_ram)

    #     # the following 6 attributes are computed by self.update()
    #     self.id, self.model, self.modelclass = None, None, None
    #     self.mode, self.lock, self.status_return_level = None, None, None
    #     self.update()

    def update(self):
        """Update precalculated values"""
        self.id = self._memory_data[pt.ID.addr]

        try:
            self.model = pt.MODELS[self._memory_data[pt.MODEL_NUMBER.addr]]
            self.modelclass = self.model[:2]
        except KeyError:
            print(("pydyn detected an unsupported motor model with model number"
                   " {}. It may just be an particularly improbable error of "
                   "transmission, or it could be a unsupported motor.\n"
                   "To support a new motor, one only need to edit the file "
                   "dynamixel/protocol.py in the source. Don't hesitate to "
                   "submit a pull request in that case. Alternatively, you can "
                   "open an issue on github with a link to your motor "
                   "documentation."
                   ).format(self._memory_data[pt.MODEL_NUMBER.addr]))
            exit()

        self.status_return_level = self._memory_data[pt.STATUS_RETURN_LEVEL.addr]

        mode_test = (self._memory_data[pt.CW_ANGLE_LIMIT.addr] ==
                     self._memory_data[pt.CCW_ANGLE_LIMIT.addr] == 0)
        self.mode = 'wheel' if mode_test else 'joint'

        self.lock = bool(self._memory_data[pt.LOCK.addr])

    def extra_addr(self):
        """address and size of extra RAM. None if no extra RAM."""
        if self.modelclass == 'MX':
            return 68, 6
        if self.modelclass == 'EX':
            return 56, 2

    def last_addr(self):
        """Last address of the RAM"""
        extra = self.extra_addr()
        if extra is None:
            return 24+26 - 1
        else:
            return extra[0]+extra[1] - 1

    # def _process_extra(self, rex):
    #     """Process the extra ram""" # TODO remove
    #     if self.modelclass == 'MX':
    #         self[68+0] = rex[0] + (rex[1] << 8)
    #         self[68+2] = rex[2]
    #         self[68+3] = rex[3] + (rex[4] << 8)
    #         self[68+5] = rex[5]
    #     if self.modelclass == 'EX':
    #         self[56+0] = rex[0] + (rex[1] << 8)

    # def _process_raw_eeprom(self, rep): # TODO remove
    #     """Return the eeprom data, with two bytes data properly computed"""
    #     assert len(rep) >= 19
    #     self[0]  = rep[0] + (rep[1] << 8)
    #     self[2]  = rep[2]
    #     self[3]  = rep[3]
    #     self[4]  = rep[4]
    #     self[5]  = rep[5]
    #     self[6]  = rep[6] + (rep[7] << 8)
    #     self[8]  = rep[8] + (rep[9] << 8)
    #     self[10] = rep[10]  # undocumented
    #     self[11] = rep[11]
    #     self[12] = rep[12]
    #     self[13] = rep[13]
    #     self[14] = rep[14] + (rep[15] << 8)
    #     self[16] = rep[16]
    #     self[17] = rep[17]
    #     self[18] = rep[18]

    #     if len(rep) >= 24:
    #         self[19] = rep[19]  # undocumented
    #         self[20] = rep[20] + (rep[21] << 8)
    #         self[22] = rep[22] + (rep[23] << 8)

    # def cache_ram(self, raw_ram):
    #     """
    #     RAM values are volatiles, they are reset each time the power is cut.
    #     Some RAM value can't change unless the user write to them. As such,
    #     any read request can use the cached values without triggering a
    #     serial communication.

    #     :param raw_ram  raw eeprom data, ie a list of 26 or 28 integers
    #                     each between 0 and 255.

    #     .. note:: While AX and RX series documents RAM up to the address 49,
    #               MX series also documents adress 68 and 69 (as two-byte
    #               consuming current value). If the ram list of value is 28,
    #               the last two values are expected to be the one at adress
    #               68 and 69.
    #     """
    #     self._process_raw_ram(raw_ram)

    # def _process_raw_ram(self, rram): # TODO remove
    #     """Process the ram data, with two bytes data properly computed"""
    #     assert len(rram) >= 26
    #     self[24+ 0] = rram[0]
    #     self[24+ 1] = rram[1]
    #     self[24+ 2] = rram[2]
    #     self[24+ 3] = rram[3]
    #     self[24+ 4] = rram[4]
    #     self[24+ 5] = rram[5]
    #     self[24+ 6] = rram[6]  + (rram[7]  << 8)
    #     self[24+ 8] = rram[8]  + (rram[9]  << 8)
    #     self[24+10] = rram[10] + (rram[11] << 8)
    #     self[24+12] = rram[12] + (rram[13] << 8)
    #     self[24+14] = rram[14] + (rram[15] << 8)
    #     self[24+16] = rram[16] + (rram[17] << 8)
    #     self[24+18] = rram[18]
    #     self[24+19] = rram[19]
    #     self[24+20] = rram[20]
    #     self[24+21] = rram[21] # undocumented
    #     self[24+22] = rram[22]
    #     self[24+23] = rram[23]
    #     self[24+24] = rram[24] + (rram[25] << 8)

    #     if len(rram) >= 28:
    #         self[24+26] = rram[26] + (rram[27] << 8)

    def _next_addr(self, addr):
        if hasattr(addr, 'addr'): # for translating Control instances.
            addr = addr.addr
        if self._memory_data[addr+1] is None:
            return addr + 2
        else:
            return addr + 1

    def __getitem__(self, addr):
        if hasattr(addr, 'addr'):
            addr = addr.addr
        return self._memory_data[addr]

    def __setitem__(self, addr, val):
        """Change memory cells values
        :param addr:  the starting address (index) a which to write the values.
        :param val:   integer or Control instance, either as single value or
                      iterable of integers, in which case values are written
                      in consecutive addr, starting at addr.
        """
        assert not hasattr(val, '__iter__')
        # if hasattr(val, '__iter__'):
        #     for v_i in val:
        #         self._set_addr(addr, v_i)
        #         addr = self._next_addr(addr)
        # else:
        self._set_addr(addr, val)

    def _set_addr(self, addr, val):
        """Set a single memory cell's value. Called by __setitem__"""
        if hasattr(val, 'addr'):
            val = val.addr
        if type(val) != int:
            raise ValueError("Motors raw values should be integers (got {})".format(type(val)))
        self._memory_data[addr] = val
        if self.save:
            self.history[addr] = val

    def long_desc(self):
        """exhaustive repr of the entire raw memory"""
        desc = '\n'.join('{:2i}: {:4i}'.format(address, value)
                         for address, value in enumerate(self._memory_data)
                         if value is not None)
        return desc + '\n'
