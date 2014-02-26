"""If you have a packet you want to analyse, drop it here"""
import env
from pydyn.ios.serialio import packet

packet_data = [255, 255, 4, 8, 0, 2, 2, 0, 0, 224, 255, 255]
packet.StatusPacket(packet_data)
