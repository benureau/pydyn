import time
import array

import ftd2xx

import pydyn.dynamixel.packet

print('Loading the robot from serial bus...')
#port = pydyn.dynamixel.get_available_ports()[0]
#print('Port found: {}'.format(port))
#io = pydyn.dynamixel.io.DynamixelIO(port, timeout = 0.1, baudrate = 1000000)
serial = ftd2xx.open()
serial.setLatencyTimer(2)
serial.setBaudRate(1000000)
serial.setTimeouts(200, 200)

# send_data = []
# for i in [1]:

def send(i, pad=False):
	packet = pydyn.dynamixel.packet.DynamixelReadDataPacket(i, 'PRESENT_POS_SPEED_LOAD')
	send_data = packet.to_bytes()

	# if pad:
	# 	send_data += array.array('B', [255]*(62-len(send_data))).tostring()
 	#print (len(send_data), [ord(c) for c in send_data])
	assert serial.write(send_data) == len(send_data)
	#io._serial.flushInput()
	data = serial.read(12)
	odata = [ord(c) for c in data]

	if len(odata) != 12:
		print (len(odata), [ord(c) for c in odata])
	return odata


if __name__ == "__main__":
	rep = 10
	start = time.time()
	for i in range(rep):
		for j in range(1, 7):
			send(j)

	dur = time.time() - start
	print('{} reads, avg: {}'.format((6*rep), dur/(6*rep)))

	# start = time.time()
	# data = send(1, pad=False)
	# print data
	# dur = time.time() - start
	# print('one: {}'.format(dur))

#serial.close()
