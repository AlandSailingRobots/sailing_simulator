#!/usr/bin/python

import socket
import sys
import select
import threading
import time
import copy
import atexit
from struct import *
import numpy as np
import core_model_sim as cms
import core_draw_sim as cds

from matplotlib import pylab as plt
from matplotlib import lines


def exit_function_py():
    if init_prog:
        threadLock.acquire()
        temp_data.set_run(0)
        threadLock.release()
        thread_draw.join()

atexit.register(exit_function_py)
init_prog = 0


class data_handler(object):
    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = 0
        self.delta_s = 0
        self.delta_r = 0
        self.phi = 0
        self.latitude = 0
        self.longitude = 0
        self.run = 1

    def set_value(self, x_, y_, theta_, delta_s_, delta_r_, phi_,
                  latitude_, longitude_):
        self.x = x_
        self.y = y_
        self.theta = theta_
        self.delta_s = delta_s_
        self.delta_r = delta_r_
        self.phi = phi_
        self.latitude = latitude_
        self.longitude = longitude_

    def set_run(self, run_):
        self.run = run_


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


class drawThread (threading.Thread):
    def __init__(self, lock_):
        threading.Thread.__init__(self)
        self.lock = lock_
        self.run_th = 1
        self.threadID = 1
        self.name = "Draw thread"
        self.counter = 1

    def run(self):
        fig = plt.figure()
        fig.subplots_adjust(top=0.8)
        ax2 = fig.add_axes([0.1, 0.1, 0.8, 0.8])
        ax2.set_xlabel('Simulation of boat')
        (ax_min_x, ax_min_y, axis_len) = (-20, -20, 40)
        while(self.run_th):

            self.lock.acquire()
            th_data = copy.deepcopy(temp_data)
            self.lock.release()
            self.run_th = th_data.run

            plt.cla()   # Clear axis
            plt.clf()   # Clear figure
            fig.subplots_adjust(top=0.8)
            ax2 = fig.add_axes([0.1, 0.1, 0.8, 0.8])
            ax2.set_xlabel('Simulation of boat %0.1f %0.1f rudder  : %0.3f\
                            lat %.5f long %.5f' %
                           (cms.wrapTo2Pi(-th_data.theta+np.pi/2)*180/np.pi,
                            cms.wrapTo2Pi(-th_data.phi+np.pi/2)*180/np.pi,
                            th_data.delta_r,
                            th_data.latitude, th_data.longitude))
            cds.draw_boat(ax2, 1, th_data.x, th_data.y,
                          th_data.theta, th_data.delta_s, th_data.delta_r)
            ax_min_x = x-axis_len/2.0
            ax_min_y = y-axis_len/2.0
            cds.draw_wind_direction(ax2, (ax_min_x+1,
                                          ax_min_y+1), axis_len, 1, phi)
            plt.axis([ax_min_x, ax_min_x+axis_len,
                      ax_min_y, ax_min_y+axis_len])
            plt.draw()
            plt.pause(0.001)

        print("Stopping Draw Thread")
        plt.close()

temp_data = data_handler()

if __name__ == '__main__':

    if len(sys.argv) == 2:
        address_prog = sys.argv[1]
        if address_prog == '-h':
            print("Usage: simulation_main [ip_address]")
            print("    ip _address    default is localhost\n")
            exit(0)
    else:
        address_prog = 'localhost'

    # Create a TCP/IP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Connect the socket to the port where the server is listening
    server_address = (address_prog, 6400)
    print('connecting to %s port %s' % server_address)
    try:
        sock.connect(server_address)
    except socket.error as e:
        print('Socket error:', e)

    s_hand = Socket_handler()
    s_hand.socket_pack()

    # multithreading management:
    threadLock = threading.Lock()
    thread_draw = drawThread(threadLock)
    init_prog = 1

    dt = 0.1

    bytes_received = 0
    data = bytearray()
    simulation = cms.simulation()
    # Set up windspeed,direction (where the wind is going in radian)
    simulation.set_wind(3, np.pi/2)

    time.sleep(0.05)
    sock.setblocking(0)
    delta_t = 0.05

    print("Start drawing thread")
    thread_draw.start()

    try:
        while(True):

            deb = time.time()
            # Handle socket receive first
            ready_to_read, ready_to_write, in_error = select.select([sock],
                                                                    [sock],
                                                                    [sock],
                                                                    0.01)

            if len(ready_to_read):
                data += sock.recv(2*2-bytes_received)
                bytes_received = len(data)
                if bytes_received is 4:
                    (command_rudder,
                     command_sheet) = s_hand.socket_unpack(data)
                    simulation.set_actuators(command_rudder, command_sheet)
                    bytes_received = 0
                    data = bytearray()

            simulation.one_loop(delta_t)
            (head, gps, ardu, wind) = simulation.get_to_socket_value()
            (x, y, theta) = simulation.get_boat().get_graph_values()
            (delta_r, delta_s,
             phi_ap, phi, latitude, longitude) = simulation.get_graph_values()

            threadLock.acquire()
            temp_data.set_value(x, y, theta, delta_s, delta_r, phi,
                                latitude, longitude)
            threadLock.release()

            s_hand.set_gps(gps[0], gps[1], gps[2], gps[3], gps[4])
            s_hand.set_windsensor(wind[1], wind[0])
            s_hand.set_compass_heading(head[0][0])
            s_hand.set_arduino(ardu[0], ardu[1], ardu[2], ardu[3])
            s_hand.socket_pack()
            ready_to_read, ready_to_write, in_error = select.select([sock],
                                                                    [sock],
                                                                    [sock],
                                                                    0.01)

            if len(ready_to_write):
                sock.sendall(s_hand.get_data_pack_send())

            dt_sleep = 0.05-(time.time()-deb)
            if dt_sleep < 0:
                dt_sleep = 0.05
            time.sleep(dt_sleep)

    except socket.error as msg:
        print("Error :", msg)

    threadLock.acquire()
    temp_data.set_run(0)
    threadLock.release()
    thread_draw.join()
