
try:
    from ioserial import DynamixelIOSerial
    serial_available = True
except ImportError:
    serial_available = False

try:
    from iovrep import DynamixelIOVRep
    vrep_available = True
except ImportError:
    vrep_available = False

if serial_available:
    DynamixelIO = DynamixelIOSerial
elif vrep_available:
    DynamixelIO = DynamixelIOVRep
else:
    raise ImportError("No I/O available (serial or vrep). Make sure pyserial or pyvrep is correctly setup.")

