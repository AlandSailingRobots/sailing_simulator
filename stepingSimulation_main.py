#!/usr/bin/python3

# Command line arguments:
# traffic: 0 or 1
# 0, simulating without traffic
# 1, simulating with traffic
# Example: simulation_main.py 1 (simulate with traffic)

# Python packages
import socket
import sys
import select
import threading
import time
import copy
import atexit
from struct import *
import numpy as np
from math import cos, sin, atan2, hypot
import queue

# Our packages
from utils import *
import core_draw_sim as cds
from simulator import Simulator
from physics_models import SimplePhysicsModel, SailingPhysicsModel, ASPirePhysicsModel, WindState
from vessel import Vessel, SailBoat, MarineTraffic
from network import Network
from data_handler import data_handler, waypoint_handler, sailBoatData, wingBoatData
from obstacle_avoidance_utils import Functions as fcn
from obstacle_avoidance_utils import *

# from drawThread import drawThread
from math import cos, sin, atan2, hypot
from matplotlib.widgets import Button
from mpl_interaction import figure_pz
import objgraph
from matplotlib.collections import LineCollection
from matplotlib import pyplot as plt
from matplotlib import lines
import matplotlib.patches as patches

import json

import LatLonMath
from draw import drawThread as dt

MESSAGE_TYPE_SAILBOAT_DATA = 0
MESSAGE_TYPE_WINGBOAT_DATA = 1
MESSAGE_TYPE_AIS_CONTACT = 2
MESSAGE_TYPE_TIS_CONTACT = 3
MESSAGE_TYPE_WINGBOAT_CMD = 4
MESSAGE_TYPE_SAILBOAT_CMD = 5
MESSAGE_TYPE_WAYPOINT_DATA = 6
MESSAGE_TYPE_ACK = 8

# Returns the milliseconds
getMillis = lambda: int(round(time.time() * 1000))


def exit_function_py():
    if init_prog:
        threadLock.acquire()
        temp_data.set_run(0)
        temp_wp.set_run(0)
        threadLock.release()
        thread_draw.join()


atexit.register(exit_function_py)
init_prog = 0

BOAT_UPDATE_MS = 100
AIS_UPDATE_MS = 500
CAMERA_FOV = 24
boatInCenter = False

temp_data = data_handler()

temp_wp = waypoint_handler()

if __name__ == '__main__':
    net = Network("localhost", 6900)

    ais_vessels = []
    data_queue = queue.Queue()
    wp_queue = queue.Queue()
    ves_queue = queue.Queue()

    if len(sys.argv) >= 2:
        configPath = sys.argv[1]
    else:
        configPath = "Simu_config_0.json"

    if len(sys.argv) == 3:
        traffic = int(sys.argv[2])
    else:
        traffic = 1

    (boat_type, sim_step, vessels, trueWind) = loadConfiguration(configPath, traffic)

    simulatedBoat = vessels[0]
    print(simulatedBoat)
    if boat_type == 0:
        message_type = MESSAGE_TYPE_SAILBOAT_DATA
        temp_data = sailBoatData()
    else:
        message_type = MESSAGE_TYPE_WINGBOAT_DATA
        temp_data = wingBoatData()

    simulator = Simulator(trueWind, 1)

    # files = []
    for i in range(0, len(vessels)):
        # files.append(open("GPS_Track_" + str(i) + ".track", 'w'))
        # files[i].write("id,latitudes,longitude,distance\n")
        simulator.addPhysicsModel(vessels[i].physicsModel())

    # multithreading management:
    threadLock = threading.Lock()
    thread_draw = dt(threadLock, boat_type, data_queue, wp_queue, ves_queue)
    init_prog = 1

    bytes_received = 0
    data = bytearray()

    time.sleep(0.01)

    print("Start drawing thread")
    thread_draw.start()
    delta_r_cmd = 0
    delta_s_cmd = 0

    (wp_lon, wp_lat, wp_dec, wp_radius, wp_prevLon,
     wp_prevLat, wp_prevDec, wp_prevRad) = (0, 0, 0, 0, 0, 0, 0, 0)

    lastSentBoatData = 0
    lastAISSent = 0

    deb = time.time()

    try:
        while (net.connected()):


            simulator.step(sim_step)


            #print("STEPED @" + str(deb-time.time()))
            deb = time.time()
            noAck = True

            while(noAck):
                net.sendBoatData(simulatedBoat, message_type)

                data = net.receiveData()

                if (len(data)):
                    if data[0] == MESSAGE_TYPE_ACK:
                        noAck = False
                #time.sleep(0.1)

            stepStart = time.time()
            data = net.receiveData()
            while (len(data) <= 0):
                data = net.receiveData()
            print("WAIT TIME " + str(time.time() - stepStart))
            print(data[0])

            if data[0] == MESSAGE_TYPE_WINGBOAT_CMD:
                (delta_r_cmd, delta_s_cmd) = net.readActuatorData(data)

            elif data[0] == MESSAGE_TYPE_SAILBOAT_CMD:
                (delta_r_cmd, delta_s_cmd) = net.readActuatorData(data)

            elif data[0] == MESSAGE_TYPE_WAYPOINT_DATA:
                (wp_lon, wp_lat, wp_dec, wp_radius, wp_prevLon,
                 wp_prevLat, wp_prevDec, wp_prevRad) = net.receiveWaypoint(data)

            """ setting the actuator to the order """
            simulatedBoat.physicsModel().setActuators(delta_s_cmd, delta_r_cmd)

            """ simulating one step """



            """ Getting boat data """
            (head, gps, wind) = fcn.get_to_socket_value(simulatedBoat)
            (x, y) = simulatedBoat.physicsModel().utmCoordinate()
            theta = simulatedBoat.physicsModel().heading()
            threadLock.acquire()

            """ filling a temporary set of data in order to draw the current state of the boat """
            if boat_type == 0:
                (delta_r, delta_s, phi, latitude, longitude) = get_graph_values(simulatedBoat, boat_type)
                temp_data.set_value(x, y, theta, delta_s, delta_r, trueWind.direction(), latitude, longitude,
                                    simulatedBoat.speed())
            else:
                (delta_r, delta_s, phi, latitude, longitude, MWAngle) = get_graph_values(simulatedBoat, boat_type)
                temp_data.set_value(x, y, theta, delta_s, delta_r, trueWind.direction(), latitude, longitude,
                                    simulatedBoat.speed(), MWAngle)

            temp_wp.set_value(wp_lon, wp_lat, wp_dec, wp_radius, wp_prevLon,
                              wp_prevLat, wp_prevDec, wp_prevRad)

            threadLock.release()

            # queues are thread safe so no need to lock
            # avoid the queues to become bigger and bigger
            if data_queue.empty():
                data_queue.put(temp_data)
            if wp_queue.empty():
                wp_queue.put(temp_wp)
            if ves_queue.empty():
                ves_queue.put(vessels)




    except socket.error as msg:
        print("Error :", msg)

    # for filePtr in files:
    #     filePtr.close()

    exit_function_py()
