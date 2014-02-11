try:
    from .ioserial import *
    serial_available = True
except ImportError:
    import traceback
    traceback.print_exc()
    serial_available = False

try:
    from .iovrep import *
    vrep_available = True
except ImportError:
    # import traceback
    # traceback.print_exc()
    vrep_available = False

if serial_available:
    #print("io is set as serial")
    DynamixelIO = DynamixelIOSerial
elif vrep_available:
    #print("io is set as a vrep socket")
    DynamixelIO = DynamixelIOVRep
else:
    raise ImportError("No I/O available (serial or vrep). Make sure pyserial or pyvrep is correctly setup.")
