import ftd2xx
import pydyn.dynamixel.packet

devices = ftd2xx.listDevices()


for i, dev in enumerate(devices):
    print(dev)
    print(ftd2xx.getDeviceInfoDetail(devnum=i))
    #print(ftd2xx.getVIDPID(devnum=i))
