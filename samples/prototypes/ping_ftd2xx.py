import ftd2xx
import pydyn.dynamixel.packet

serial = ftd2xx.open()
serial.setLatencyTimer(2)
serial.setBaudRate(1000000)
serial.setTimeouts(10, 10)

ping = pydyn.dynamixel.packet.DynamixelPingPacket(254)
serial.write(ping.to_bytes())
data = serial.read(1000)

print (len(data), [ord(c) for c in data])

serial.close()