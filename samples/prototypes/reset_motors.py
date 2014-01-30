import ftd2xx
import pydyn.dynamixel.packet

serial = ftd2xx.open(dev=0)
serial.setLatencyTimer(2)
serial.setBaudRate(1000000)
serial.setTimeouts(10, 10)

reset = pydyn.dynamixel.packet.DynamixelInstructionPacket(52, 'RESET')
serial.write(reset.to_bytes())
data = serial.read(1000)

print (len(data), [ord(c) for c in data])

serial.close()
