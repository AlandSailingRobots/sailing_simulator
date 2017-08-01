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
import core_draw_sim as cds
from simulator import Simulator
from physics_models import SimplePhysicsModel,SailingPhysicsModel, WindState
from vessel import Vessel,SailBoat, MarineTraffic
from network import Network

from math import cos, sin, atan2, hypot

from matplotlib import pylab as plt
from matplotlib import lines

import json

import LatLonMath


# Returns the milliseconds
getMillis = lambda: int(round(time.time() * 1000))

def exit_function_py():
    if init_prog:
        threadLock.acquire()
        temp_data.set_run(0)
        threadLock.release()
        thread_draw.join()

atexit.register(exit_function_py)
init_prog = 0



BOAT_UPDATE_MS = 100
AIS_UPDATE_MS = 500
CAMERA_ANGLE = 24


def wrapTo2Pi(theta):
    if theta < 0:
        theta += 2*np.pi
    theta = theta % (2*np.pi)
    return theta


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
        self.speed = 0

    def set_value(self, x_, y_, theta_, delta_s_, delta_r_, phi_,
                  latitude_, longitude_, speed_):
        self.x = x_
        self.y = y_
        self.theta = theta_
        self.delta_s = delta_s_
        self.delta_r = delta_r_
        self.phi = phi_
        self.latitude = latitude_
        self.longitude = longitude_
        self.speed = speed_

    def set_run(self, run_):
        self.run = run_

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
            ax2.set_xlabel('Simulation of boat %0.1f %0.1f speed:%0.1f m/s rudder:%0.3f\
lat %.5f long %.5f ' %
                           (wrapTo2Pi(-th_data.theta+np.pi/2)*180/np.pi,
                            wrapTo2Pi(-th_data.phi+np.pi/2)*180/np.pi,
                            th_data.speed,
                            th_data.delta_r,
                            th_data.latitude, th_data.longitude))
            cds.draw_boat(ax2, 1, th_data.x, th_data.y,
                          th_data.theta, th_data.delta_r, th_data.delta_s)
            ax_min_x = x-axis_len/2.0
            ax_min_y = y-axis_len/2.0
            cds.draw_wind_direction(ax2, (ax_min_x+1, ax_min_y+1), axis_len, 1, th_data.phi)
            plt.axis([ax_min_x, ax_min_x+axis_len,
                      ax_min_y, ax_min_y+axis_len])
            plt.draw()
            plt.pause(0.001)

        print("Stopping Draw Thread")
        plt.close()


def wrapTo2Pi(theta):
    if theta < 0:
        theta += 2*np.pi
    theta = theta % (2*np.pi)
    return theta

def order_to_deg(command_rudder, command_sheet):
    if command_rudder > 8000 or command_rudder < 3000:
        command_sheet = 4215
        command_rudder = 5520
    return ((command_rudder-5520)*(np.pi/6.0)/1500.0,
            (command_sheet-4215)*(np.pi/-6.165)/900.0)


def get_to_socket_value( sailBoat ):
    heading = sailBoat.heading()
    (lat, lon) = sailBoat.position()
    course = sailBoat.course()
    speed =  sailBoat.speed()

    gps = (lat, lon, course, heading, speed)

    windsensor = ( sailBoat.apparentWind().speed(), sailBoat.apparentWind().direction() )
    return (heading, gps, windsensor)

def get_graph_values( sailBoat ):
    (sail, rudder) = sailBoat.sailAndRudder()
    phi_ap = sailBoat.apparentWind().direction()

    sigma = cos( phi_ap ) + cos( sail )
    if (sigma < 0):
        sail = np.pi + phi_ap
    else:
        if sin(phi_ap)is not 0:
            sail = -np.sign( sin(phi_ap) ) * abs( sail )
    (lat, lon) = sailBoat.position()
    return ( rudder, sail, phi_ap, lat, lon )

def wrapAngle( angle ):
    while angle < 0 or angle >= 360:
        if angle < 0:
            angle += 360
        else:
            angle -= 360

    return angle

def getBearing( asv, vessel ):
    (asvLat, asvLon) = asv.position()
    (vesselLat, vesslLon) = asv.position()

    boatLatitudeInRadian = np.deg2rad(asvLat)
    waypointLatitudeInRadian = np.deg2rad(vesselLat)
    deltaLongitudeRadian = np.deg2rad(vesslLon - asvLon)

    y_coordinate = sin(deltaLongitudeRadian) * cos(waypointLatitudeInRadian)
    x_coordinate = cos(boatLatitudeInRadian)* sin(waypointLatitudeInRadian) - sin(boatLatitudeInRadian) * cos(waypointLatitudeInRadian) * cos(deltaLongitudeRadian)

    bearingToWaypointInRadian = atan2(y_coordinate, x_coordinate)
    bearingToWaypoint = np.rad2deg(bearingToWaypointInRadian)
    return wrapAngle(bearingToWaypoint)

def getBearingDiff( h1, h2 ):
    diff = h2 - h1
    absDiff = abs( diff )

    if (absDiff <= 180):
        if absDiff == 180:
            return absdiff
        else:
            return diff

    elif (h2 > h1):
        return absDiff - 360
    else:
	    return 360 - absDiff

def boatInVisualRange( asv, vessel):
    bearing = getBearing(asv, vessel)

    bearingDiff = abs( getBearingDiff(asv.heading(), bearing) )

    if bearingDiff < CAMERA_ANGLE:
        return True
    return False

def loadConfiguration(configPath):
    global AIS_UPDATE_MS
    global BOAT_UPDATE_MS

    with open(configPath) as data_file:
        config = json.load(data_file)

    latOrigin = config["lat_origin"]
    lonOrigin = config["lon_origin"]

    if config.get("boat_update_ms"):
        BOAT_UPDATE_MS = config["boat_update_ms"];

    if config.get("ais_update_ms"):
        AIS_UPDATE_MS = config["ais_update_ms"];

    print("Boat Update ms: " + str(BOAT_UPDATE_MS) + " AIS Update ms: " + str(AIS_UPDATE_MS))

    vessels = []

    trueWindDir = wrapTo2Pi(np.deg2rad(config["wind_direction"] - 90))
    print ("True Wind:" + str(trueWindDir))
    trueWindSpeed = config["wind_speed"]

    vessels.append(SailBoat( SailingPhysicsModel(), latOrigin, lonOrigin, 0, 0 ))

    # Load Marine Traffic
    for marineVessel in config["traffic"]:
        id = marineVessel["mmsi"];
        lat = marineVessel["lat_origin"]
        lon = marineVessel["lon_origin"]
        heading = wrapTo2Pi(np.deg2rad(marineVessel["heading"] - 90))
        speed = marineVessel["speed"]
        length = marineVessel["length"]
        beam = marineVessel["beam"]
        vessels.append(MarineTraffic(SimplePhysicsModel(heading, speed), lat, lon, heading, speed, id, length, beam))

    return ( vessels, WindState( trueWindDir, trueWindSpeed ) )

temp_data = data_handler()

if __name__ == '__main__':
    net = Network( "localhost", 6900 )

    configPath = "config.json"

    if len(sys.argv) == 2:
        configPath = sys.argv[1]

    ( vessels, trueWind ) = loadConfiguration(configPath)
    simulatedBoat = vessels[0]
    print(simulatedBoat);

    simulator = Simulator( trueWind, 1 )

    files = []
    for i in range(0, len(vessels)):
        files.append(open("GPS_Track_" + str(i) + ".track", 'w'))
        files[i].write("id,latitudes,longitude,distance\n")
        simulator.addPhysicsModel( vessels[i].physicsModel() )

    # multithreading management:
    threadLock = threading.Lock()
    thread_draw = drawThread(threadLock)
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

            (command_rudder, command_sheet) = net.readActuatorData()
            (delta_r, delta_s) = order_to_deg(command_rudder, command_sheet)

            simulatedBoat.physicsModel().setActuators( delta_s, delta_r )

            # TODO - Jordan: Make this a variable step, so we aren't at a fixed rate of simulation
            simulator.step( 0.05 )

            millis = getMillis();

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


            (head, gps, wind) = get_to_socket_value( simulatedBoat )
            (x, y) = simulatedBoat.physicsModel().utmCoordinate()
            theta = simulatedBoat.physicsModel().heading()
            (delta_r, delta_s, phi, latitude, longitude) = get_graph_values( simulatedBoat )

            threadLock.acquire()
            temp_data.set_value(x, y, theta, delta_s, delta_r, trueWind.direction(),
                                latitude, longitude, simulatedBoat.speed())
            threadLock.release()

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
    threadLock.release()
    thread_draw.join()
