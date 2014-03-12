
from pyftdi.pyftdi import ftdi
vps=[(0x0403,0x6001)]
devs=ftdi.Ftdi.find_all(vps)
print devs

pid, vid, serial_id, interface, description = devs[0]

serial = ftdi.Ftdi()
serial.open(pid, vid, interface, serial=serial_id, description=description)
serial.close()