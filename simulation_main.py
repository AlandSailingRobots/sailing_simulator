#!/usr/bin/python

# Command line arguments:
# boat_type: 0 or 1
# 0, simulating with Janet
# 1, simulatiing with ASPire
# traffic: 0 or 1
# 0, simulating without traffic
# 1, simulating with traffic
# Example: simulation_main.py 0 1 (simulate Janet with traffic)

# Python packages
import socket
import sys
import select
import threading
import time
import copy
import atexit
from struct import *
from math import cos, sin, atan2, hypot
import queue

# Our packages
import utils
import numpy as np
import core_draw_sim as cds
from simulator import Simulator
from physics_models import SimplePhysicsModel,SailingPhysicsModel, WindState, ASPirePhysicsModel
from vessel import Vessel, SailBoat, MarineTraffic
from network import Network
from utils import Functions as fcn
from data_handler import data_handler, waypoint_handler, sailBoatData, wingBoatData
import LatLonMath
from draw import drawThread as dt


MESSAGE_TYPE_SAILBOAT_DATA = 0
MESSAGE_TYPE_WINGBOAT_DATA = 1
MESSAGE_TYPE_AIS_CONTACT   = 2
MESSAGE_TYPE_TIS_CONTACT   = 3

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
CAMERA_ANGLE = 24

temp_wp = waypoint_handler()

if __name__ == '__main__':
    # Change so we can have a variable with an ip address
    net = Network( "localhost", 6900 )

    configPath = "config.json"

    ais_vessels = []
    data_queue = queue.Queue()
    wp_queue = queue.Queue()
    ves_queue = queue.Queue()

    # if len(sys.argv) == 2:
    #     configPath = sys.argv[1]

    if len(sys.argv) == 3:
        boat_type = int(sys.argv[1])
        traffic = int(sys.argv[2])
    else:
        boat_type = 0
        traffic = 1

    ( boat_type, vessels, trueWind ) = utils.loadConfiguration(configPath, traffic, boat_type)

    simulatedBoat = vessels[0]
    print(simulatedBoat)
    if boat_type == 0:
        message_type = MESSAGE_TYPE_SAILBOAT_DATA
        temp_data = sailBoatData()
    else:
        message_type = MESSAGE_TYPE_WINGBOAT_DATA
        temp_data = wingBoatData()
    simulator = Simulator( trueWind, 1 )

    files = []
    for i in range(0, len(vessels)):
        files.append(open("GPS_Track_" + str(i) + ".track", 'w'))
        files[i].write("id,latitudes,longitude,distance\n")
        simulator.addPhysicsModel( vessels[i].physicsModel() )

    # multithreading management:
    threadLock = threading.Lock()
    thread_draw = dt(threadLock, boat_type, data_queue, wp_queue, ves_queue)
    init_prog = 1

    dt = 0.1

    bytes_received = 0
    data = bytearray()

    time.sleep(0.05)
    delta_t = 0.05

    print("Start drawing thread")
    thread_draw.start()
    delta_r = 0
    delta_s = 0

    lastSentBoatData = 0
    lastAISSent = 0

    try:
        while( net.connected() ):

            deb = time.time()

            if boat_type == 0:
                (command_rudder, command_sheet) = net.readActuatorData()
                (delta_r, delta_s) = fcn.order_to_deg(command_rudder, command_sheet)
            else:
                net.readActuatorData()  # This line have to be here in order for net.receiveWaypoint() to work!
                (delta_r,delta_s) = (0,0)

            (wp_lon, wp_lat, wp_dec, wp_radius, wp_prevLon,
             wp_prevLat, wp_prevDec, wp_prevRad) = net.receiveWaypoint()
            simulatedBoat.physicsModel().setActuators( delta_s, delta_r )

            # TODO - Jordan: Make this a variable step, so we aren't at a fixed rate of simulation
            # 0.05 is probably a good value, smaller value is to quick for us to receive and unpack the waypoint data
            simulator.step( 0.05 )

            millis = getMillis()

            # Send the boat data
            if millis > lastSentBoatData + BOAT_UPDATE_MS:
                net.sendBoatData( simulatedBoat )
                lastSentBoatData = millis

            # Send AIS data
            if millis > lastAISSent + AIS_UPDATE_MS:
                for i in range( 1, len(vessels) ):
                    if vessels[i].id() < 100000000 and boatInVisualRange(vessels[0], vessels[i]):
                        net.sendVisualContact( vessels[i] )
                    else:
                        net.sendAISContact( vessels[i] )
                lastAISSent = millis

            (head, gps, wind) = fcn.get_to_socket_value( simulatedBoat )
            (x, y) = simulatedBoat.physicsModel().utmCoordinate()
            theta = simulatedBoat.physicsModel().heading()

            if boat_type == 0:
                (delta_r, delta_s, phi, latitude, longitude) = utils.get_graph_values(simulatedBoat, boat_type)
                temp_data.set_value(x, y, theta, delta_s, delta_r, trueWind.direction(),latitude, longitude, simulatedBoat.speed())
            else:
                (delta_r, delta_s, phi, latitude, longitude, MWAngle) = utils.get_graph_values(simulatedBoat, boat_type)
                temp_data.set_value(x, y, theta, delta_s, delta_r, trueWind.direction(),latitude, longitude, simulatedBoat.speed(), MWAngle)

            temp_wp.set_value(wp_lon, wp_lat, wp_dec, wp_radius, wp_prevLon,
                              wp_prevLat, wp_prevDec, wp_prevRad)

            # queues are thread safe so no need to lock
            data_queue.put(temp_data)
            wp_queue.put(temp_wp)
            ves_queue.put(vessels)

            (asvLat, asvLon) = vessels[0].position()

            # Log marine traffic
            for i in range( 0, len(vessels) ):
                (lat, lon) = vessels[i].position()
                distance = LatLonMath.distanceKM(lat, lon, asvLat, asvLon)
                files[i].write("0," + str(lat) + "," + str(lon) + "," + str(distance) + "\n")
            dt_sleep = 0.05-(time.time()-deb)
            if dt_sleep < 0:
                dt_sleep = 0.05
            time.sleep(dt_sleep)

    except socket.error as msg:
        print("Error :", msg)

    for filePtr in files:
        filePtr.close()

    threadLock.acquire()
    temp_data.set_run(0)
    temp_wp.set_run(0)
    threadLock.release()
    thread_draw.join()
