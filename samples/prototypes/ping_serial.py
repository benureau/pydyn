import time
import array
import serial
import pydyn.dynamixel.packet

ser = serial.Serial('/dev/cu.usbmodem411', baudrate=1000000, timeout=0.02)
#ser.open()
# tossmode = [116, 0x0D]
# tossmode = array.array('B', tossmode).tostring()
# ser.write(tossmode)

#tossmode = [116, 0x0D]
#tossmode = array.array('B', tossmode).tostring()
#ser.write('t\r')
#time.sleep(0.5)
#data = ser.read(100)
#print data
# tossmode = [116, 0x0D]
# tossmode = array.array('B', tossmode).tostring()
# ser.write(tossmode)



mids = []

for i in range(100):
    ping = pydyn.dynamixel.packet.DynamixelPingPacket(i)
    ser.write(ping.to_bytes())
    data = ser.read(6)
    if len(data) > 0:
        print(i)
        print([ord(e) for e in data])
        mids.append(i)

print(mids)

ser.close()