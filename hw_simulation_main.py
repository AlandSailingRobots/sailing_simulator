#!/usr/bin/python3

from ViritualBoat import ViritualBoat

import time
import math as m
import sys
import queue
import threading

from obstacle_avoidance_utils import *
from simulator import Simulator
from data_handler import data_handler, waypoint_handler, sailBoatData, wingBoatData
from draw import drawThread as dt
from obstacle_avoidance_utils import Functions as fcn


getMillis = lambda: int(round(time.time() * 1000))

BOAT_UPDATE_MS = 100



temp_data = wingBoatData()

temp_wp = waypoint_handler()



def initGrafics():
    global delta_r_cmd, delta_s_cmd, data_queue, wp_queue, ves_queue, threadLock, thread_draw, init_prog, wp_lon, wp_lat, wp_dec, wp_radius, wp_prevLon,wp_prevLat, wp_prevDec, wp_prevRad
    data_queue = queue.Queue()
    wp_queue = queue.Queue()
    ves_queue = queue.Queue()

    threadLock = threading.Lock()
    thread_draw = dt(threadLock, boat_type, data_queue, wp_queue, ves_queue)
    init_prog = 1

    print("Start drawing thread")
    thread_draw.start()
    delta_r_cmd = 0
    delta_s_cmd = 0

    (wp_lon, wp_lat, wp_dec, wp_radius, wp_prevLon,
     wp_prevLat, wp_prevDec, wp_prevRad) = (0, 0, 0, 0, 0, 0, 0, 0)



def updateGrafics():
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

if __name__ == '__main__':



    if len(sys.argv) >= 2:
        configPath = sys.argv[1]
    else:
        configPath = "Simu_config_0.json"

    if len(sys.argv) >= 3:
       ip = sys.argv[2]
    else:
        ip = "10.112.147.10" #IP of ASPire1 in ASR-VPN-network
    if len(sys.argv) >=4:
        serPort = sys.argv[3]
    else:
        serPort = "/dev/ttyUSB0" #normal port of arduino nano runnning on arch-linux
    if len(sys.argv) >=5:
        if sys.argv[4] == "1":
            graficsOn = True
    else:
        graficsOn = False

    traffic = 0
    ( boat_type, sim_step,vessels, trueWind ) = loadConfiguration(configPath, traffic)



    simulatedBoat = vessels[0]
    latLongBoat = simulatedBoat.position()
    virtualBoat = ViritualBoat(latLongBoat[0], latLongBoat[1],ip , serPort)
    virtualBoat.start()
    virtualBoat.startActuators()

    simulator = Simulator(trueWind, 1)
    simulator.addPhysicsModel(simulatedBoat.physicsModel())

    if (graficsOn):
        initGrafics()


    lastSentBoatData = 0

    while(True):

        deb = time.time()

        (rudder, tailWing) = virtualBoat.getActuatorData()
        simulatedBoat.physicsModel().setActuators(tailWing, rudder)
        simulator.step(sim_step)
        virtualBoat.setNavigationParameters(simulatedBoat)


        if (graficsOn):
            updateGrafics()


        dt_sleep = sim_step - (time.time() - deb)
        if dt_sleep < 0:
            dt_sleep = sim_step
        time.sleep(dt_sleep)
        #time.sleep(0.05)



