import socket
import sys
import numpy as np
from struct import *


class Socket_handler(object):
    def __init__(self):
        print("value")
        self.data_pack_receive = 0
        self.data_pack_send = 0
        self.send_format = '=8f4H12HBB'
        self.receive_format = '=HH'

        self.compass = ((0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0, 0), 0x19)
        self.arduino = (0, 0, 0, 0, 0x07)
        self.gps = (60.20, 19.14, 0, 0, 0)
        self.windsensor = (0, 0, 0)

    def set_compass_heading(self, heading):
        self.compass[0][0] = heading

    def set_gps(self, latitude, longitude, course_real, course_magn,
                speed_knot):
        self.gps = (latitude, longitude, course_real, course_magn, speed_knot)

    def set_arduino(self, pressure, rudder, sheet, battery, address=0x07):
        self.arduino = (pressure, rudder, sheet, battery, address)

    def set_windsensor(self, wind_direction, wind_speed, temperature=24):
        self.windsensor = (wind_direction, wind_speed, temperature)

    def socket_pack(self):
        self.data_socket_send = pack(self.send_format, self.gps[0],
                                     self.gps[1], self.gps[2], self.gps[3],
                                     self.gps[4],
                                     self.windsensor[0], self.windsensor[1],
                                     self.windsensor[2],
                                     self.arduino[0], self.arduino[1],
                                     self.arduino[2], self.arduino[3],
                                     self.compass[0][0],
                                     self.compass[0][1], self.compass[0][2],
                                     self.compass[1][0], self.compass[1][1],
                                     self.compass[1][2], self.compass[2][0],
                                     self.compass[2][1], self.compass[2][2],
                                     self.compass[3][0], self.compass[3][1],
                                     self.compass[3][2], self.compass[4],
                                     self.arduino[4])

    def get_data_pack_send(self):
        return self.data_socket_send

# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.setblocking(0)

# Connect the socket to the port where the server is listening
server_address = ('localhost', 6400)
print('connecting to %s port %s' % server_address)
try:
    sock.connect(server_address)
except socket.error as e:
    print('Socket error:', e)


s_hand = Socket_handler()
s_hand.socket_pack()


sock.sendall(s_hand.get_data_pack_send())
print(s_hand.get_data_pack_send())
