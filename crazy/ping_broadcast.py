import pydyn.dynamixel
import pydyn.dynamixel.io
import pydyn.dynamixel.packet

print('Loading the robot from serial bus...')
port = pydyn.dynamixel.get_available_ports()[0]
print('Port found: {}'.format(port))
io = pydyn.dynamixel.io.DynamixelIO(port, timeout = 0.1, baudrate = 1000000)

ping = pydyn.dynamixel.packet.DynamixelPingPacket(254)
io._send_packet(ping, receive_status_packet=False)
data = io._serial.read(size = 1000)

print (len(data), [ord(c) for c in data])
assert len(data) % 6 == 0
