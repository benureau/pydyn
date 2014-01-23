import ftd2xx
import pydyn.dynamixel.packet

serial = ftd2xx.open()
#serial.setLatencyTimer(2)
serial.setBaudRate(1000000)
serial.setTimeouts(200, 200)


packet = pydyn.dynamixel.packet.DynamixelReadDataPacket(1, 'PRESENT_POS_SPEED_LOAD')
serial.write(packet.to_bytes())
data = serial.read(12)

print([ord(d) for d in data])

serial.close()