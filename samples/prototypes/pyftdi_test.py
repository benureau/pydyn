
from pyftdi.pyftdi.ftdi import *
vps=[(0x0403,0x6001)]
devs=Ftdi.find_all(vps)
print devs

serial = Ftdi()
serial = serial.open(*devs[0])
serial.close()