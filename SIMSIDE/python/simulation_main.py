import socket
import sys
import numpy as np
from struct import *
import core_model_sim as cms
import core_draw_sim as cds
import time
import matplotlib.pylab as plt
import matplotlib.lines as lines
import select


class Socket_handler(object):
    def __init__(self):
        print("value")
        self.data_pack_receive = 0
        self.data_pack_send = 0
        self.send_format = '=8f4H12HBB'
        self.receive_format = '=HH'

        self.compass = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0, 0],
                        int(0x19)]
        self.arduino = (0, 0, 0, 0, int(0x07))
        self.gps = (60.20, 19.14, 0, 0, 0)
        self.windsensor = (0, 0, 0)

    def set_compass_heading(self, heading):
        self.compass[0][0] = int(heading*10)
        print("heading cmp", heading)

    def set_gps(self, latitude, longitude, course_real, course_magn,
                speed_knot):
        self.gps = (latitude, longitude, course_real, course_magn, speed_knot)

    def set_arduino(self, pressure, rudder, sheet, battery, address=0x07):
        if rudder < 0:
            rudder = 0
        self.arduino = (int(pressure), int(rudder), int(sheet), int(battery),
                        int(address))

    def set_windsensor(self, wind_direction, wind_speed, temperature=24):
        self.windsensor = (wind_direction, wind_speed, int(temperature))

    def socket_pack(self):
        # print(self.arduino, self.gps, self.windsensor, self.compass)
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
                                     int(self.arduino[4]))

    def socket_unpack(self, data):
        return unpack(self.receive_format, data)

    def get_data_pack_send(self):
        return self.data_socket_send


if __name__ == '__main__':

    # Create a TCP/IP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Connect the socket to the port where the server is listening
    server_address = ('localhost', 6400)
    print('connecting to %s port %s' % server_address)
    try:
        sock.connect(server_address)
    except socket.error as e:
        print('Socket error:', e)

    s_hand = Socket_handler()
    s_hand.socket_pack()

    dt = 0.1

    bytes_received = 0
    data = bytearray()
    simulation = cms.simulation()
    # Set up windspeed,direction (where the wind is going in radian)
    simulation.set_wind(3, np.pi/2)

    fig = plt.figure()
    fig.subplots_adjust(top=0.8)
    ax2 = fig.add_axes([0.1, 0.1, 0.8, 0.8])
    ax2.set_xlabel('Simulation of boat')
    (ax_min_x, ax_min_y, axis_len) = (-20, -20, 40)

    time.sleep(0.1)
    sock.setblocking(0)
    delta_t = 0.05

    while(True):

        deb = time.time()
        # Handle socket receive first
        ready_to_read, ready_to_write, in_error = select.select([sock], [sock],
                                                                [sock], 0.01)

        if len(ready_to_read):
            data += sock.recv(2*2-bytes_received)
            bytes_received = len(data)
            if bytes_received is 4:
                (command_rudder, command_sheet) = s_hand.socket_unpack(data)
                simulation.set_actuators(command_rudder, command_sheet)
                bytes_received = 0
                data = bytearray()

        simulation.one_loop(delta_t)
        (head, gps, ardu, wind) = simulation.get_to_socket_value()
        (x, y, theta) = simulation.get_boat().get_graph_values()
        (delta_r, delta_s,
         phi_ap, phi, latitude, longitude) = simulation.get_graph_values()

        # plt.cla()   # Clear axis
        # plt.clf()   # Clear figure
        # fig.subplots_adjust(top=0.8)
        # ax2 = fig.add_axes([0.1, 0.1, 0.8, 0.8])
        # ax2.set_xlabel('Simulation of boat %0.1f %0.1f rudder  : %0.3f\
        #                lat %.5f long %.5f' %
        #               (cms.wrapTo2Pi(-theta+np.pi/2)*180/np.pi,
        #                cms.wrapTo2Pi(-phi+np.pi/2)*180/np.pi,
        #                delta_r,
        #                latitude, longitude))
        # cds.draw_boat(ax2, 1, x, y, theta, delta_s, delta_r)
        # ax_min_x = x-axis_len/2.0
        # ax_min_y = y-axis_len/2.0
        # cds.draw_wind_direction(ax2, (ax_min_x+1,
        #                              ax_min_y+1), axis_len, 1, phi)
        # plt.axis([ax_min_x, ax_min_x+axis_len, ax_min_y, ax_min_y+axis_len])
        # plt.draw()
        # plt.pause(0.001)

        s_hand.set_gps(gps[0], gps[1], gps[2], gps[3], gps[4])
        s_hand.set_windsensor(wind[1], wind[0])
        s_hand.set_compass_heading(head[0][0])
        s_hand.set_arduino(ardu[0], ardu[1], ardu[2], ardu[3])
        s_hand.socket_pack()
        ready_to_read, ready_to_write, in_error = select.select([sock], [sock],
                                                                [sock], 0.01)

        if len(ready_to_write):
            sock.sendall(s_hand.get_data_pack_send())
        print("Time :", time.time()-deb)
        time.sleep(0.05)
