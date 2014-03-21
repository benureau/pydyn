"""
Motor memory. Keeps a cached image of the motor memory.
It is only as uptodate as the last read of each address.
"""
import numbers

from ..refs import protocol as pt
from ..refs.conversions import MODEL_NAMES
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
            self._memory_data = list(memory)
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

    def update(self):
        """Update precalculated values"""
        self.id = self._memory_data[pt.ID.addr]

        try:
            self.model = MODEL_NAMES[self._memory_data[pt.MODEL_NUMBER.addr]]
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
        if not isinstance(val, numbers.Integral):
            raise ValueError("Motors bytes values should be integers (got {} of type {})".format(val, type(val)))
        self._memory_data[addr] = val
        if self.save:
            self.history[addr] = val

    def long_desc(self):
        """exhaustive repr of the entire raw memory"""
        desc = '\n'.join('{:2i}: {:4i}'.format(address, value)
                         for address, value in enumerate(self._memory_data)
                         if value is not None)
        return desc + '\n'
