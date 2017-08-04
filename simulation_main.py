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
from data_handler import data_handler, waypoint_handler
from simulator import Simulator
from physics_models import SimplePhysicsModel,SailingPhysicsModel, WindState
from vessel import Vessel,SailBoat, MarineTraffic
from network import Network
from mathFcns import Functions as fcn
# from drawThread import drawThread
from math import cos, sin, atan2, hypot
from matplotlib.widgets import Button


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

class zoom(object):
    length = 0.008

    def zoomout(self, event):
        if self.length < 0.04:
            self.length += 0.001
        print(self.length)

    def zoomin(self, event):
        if self.length > 0.005:
            self.length -= 0.001
        print(self.length)


class drawThread (threading.Thread):
    def __init__(self, lock_):
        threading.Thread.__init__(self)
        self.lock = lock_
        self.run_th = 1
        self.threadID = 1
        self.name = "Draw thread"
        self.counter = 1



    def run(self):
        zoom_ = zoom()
        print(zoom_.length)
        axis_length = 0.008
        fig = plt.figure(figsize=(9, 9))
        fig.patch.set_facecolor('teal')
        fig.subplots_adjust(top=0.8)
        ax2 = fig.add_axes([0.1, 0.1, 0.7, 0.8])
        ax2.set_xlabel('Simulation of boat')
        ax2.patch.set_facecolor('lightblue')
        th_data = copy.deepcopy(temp_data)
        latprev = th_data.latitude
        lonprev = th_data.longitude
        lines = []
        prevWPlongitude = 0
        prevWPlatitude = 0

        textbox = fig.text(0.7, 0.7, '')
        zoomtext = fig.text(0.815, 0.5,'ZOOM')
        zoomtext.set_color('white')

        ax_out = fig.add_axes([0.812, 0.44, 0.08, 0.05])
        ax_in = fig.add_axes([0.902, 0.44, 0.08, 0.05])
        bzoomout = Button(ax_out, 'Out')
        bzoomin = Button(ax_in, 'In')

        bzoomout.on_clicked(zoom_.zoomout)
        bzoomin.on_clicked(zoom_.zoomin)

        windtext = fig.text(0.815, 0.66, 'Wind Direction')
        windtext.set_color('white')
        wind_ax = fig.add_axes([0.82,0.55,0.1,0.1])
        wind_ax.set_axis_off()
        cds.draw_wind_direction(wind_ax, (0, -0.5), 1, 0.3, th_data.phi)

        while(self.run_th):
            axis_length = zoom_.length
            self.lock.acquire()
            th_data = copy.deepcopy(temp_data)
            th_wp = copy.deepcopy(temp_wp)
            self.lock.release()
            self.run_th = th_data.run
            (ax_min_x, ax_min_y, axis_len) = (th_data.longitude-axis_length/2, th_data.latitude-axis_length/2, axis_length)

            btw = fcn.getBTW([th_data.longitude, th_data.latitude], [th_wp.lon, th_wp.lat])
            dtw = fcn.getDTW([th_data.longitude, th_data.latitude], [th_wp.lon, th_wp.lat])

            textstr = "ASV STATE\nLon:          %.5f\nLat:           %.5f\nHeading:  %.2f\nSpeed:       %.2f" % (th_data.longitude, th_data.latitude, wrapTo2Pi(-th_data.theta+np.pi/2)*180/np.pi, th_data.speed)
            textstr += "\n____________________\n\nWAYPOINT\nBearing:   %.2f\nDistance:  %.2f\nRadius:    %d" % (btw, dtw, th_wp.rad)

            # plt.cla()   # Clear axis
            # plt.clf()   # Clear figure

            fig.subplots_adjust(top=0.8)
            ax2 = fig.add_axes([0.1, 0.1, 0.7, 0.8])

            if th_wp.lon != prevWPlongitude:
                if th_wp.lon > 0.01 and th_wp.lon < 1e3:
                    if prevWPlongitude != 0:
                        cds.draw_line(ax2, [th_wp.lon, th_wp.lat], [prevWPlongitude, prevWPlatitude], 'k')
                    cds.draw_circle(ax2, 1, th_wp.lon, th_wp.lat, th_wp.rad)
                    prevWPlongitude = th_wp.lon
                    prevWPlatitude = th_wp.lat

            ax_min_x = th_data.longitude-axis_len/2.0
            ax_min_y = th_data.latitude-axis_len/2.0
            # cds.draw_track(ax2, lines, th_data.longitude, th_data.latitude)
            # cds.draw_boat(ax2, 0.0002, th_data.longitude, th_data.latitude,
            #               th_data.theta, th_data.delta_r, th_data.delta_s)
            # cds.draw_wind_direction(ax2, (ax_min_x+0.0001, ax_min_y+0.0001), axis_len, 0.0002, th_data.phi)
            cds.draw_line(ax2, [th_data.longitude, th_data.latitude], [lonprev, latprev], 'r')
            # ax2.text(0.05, 0.95,textstr)
            plt.axis([ax_min_x, ax_min_x+axis_len,
                      ax_min_y, ax_min_y+axis_len])
            plt.draw()
            fig.texts.remove(textbox)
            textbox = fig.text(0.82, 0.7, textstr,bbox=dict(edgecolor='white', facecolor='teal'))
            textbox.set_color('white')
            # bzoomout.on_clicked(zoom_.zoomout)
            # bzoomin.on_clicked(zoom_.zoomin)
            # plt.show()
            plt.pause(0.001)
            lonprev = th_data.longitude
            latprev = th_data.latitude

        print("Stopping Draw Thread")
        plt.close()


def loadConfiguration(configPath):
    global AIS_UPDATE_MS
    global BOAT_UPDATE_MS

    with open(configPath) as data_file:
        config = json.load(data_file)

    latOrigin = config["lat_origin"]
    lonOrigin = config["lon_origin"]

    if config.get("boat_update_ms"):
        BOAT_UPDATE_MS = config["boat_update_ms"]

    if config.get("ais_update_ms"):
        AIS_UPDATE_MS = config["ais_update_ms"]

    print("Boat Update ms: " + str(BOAT_UPDATE_MS) + " AIS Update ms: " + str(AIS_UPDATE_MS))

    vessels = []

    trueWindDir = wrapTo2Pi(np.deg2rad(config["wind_direction"] - 90))
    print ("True Wind:" + str(trueWindDir))
    trueWindSpeed = config["wind_speed"]

    vessels.append(SailBoat( SailingPhysicsModel(), latOrigin, lonOrigin, 0, 0 ))

    # Load Marine Traffic
    for marineVessel in config["traffic"]:
        id = marineVessel["mmsi"]
        lat = marineVessel["lat_origin"]
        lon = marineVessel["lon_origin"]
        heading = wrapTo2Pi(np.deg2rad(marineVessel["heading"] - 90))
        speed = marineVessel["speed"]
        length = marineVessel["length"]
        beam = marineVessel["beam"]
        vessels.append(MarineTraffic(SimplePhysicsModel(heading, speed), lat, lon, heading, speed, id, length, beam))

    return ( vessels, WindState( trueWindDir, trueWindSpeed ) )


temp_data = data_handler()
temp_wp = waypoint_handler()

if __name__ == '__main__':
    # global temp_wp, temp_data
    net = Network( "localhost", 6900 )

    configPath = "config.json"

    if len(sys.argv) == 2:
        configPath = sys.argv[1]

    ( vessels, trueWind ) = loadConfiguration(configPath)
    simulatedBoat = vessels[0]
    print(simulatedBoat)

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
            (delta_r, delta_s) = fcn.order_to_deg(command_rudder, command_sheet)

            (wp_lon, wp_lat, wp_dec, wp_radius, wp_prevLon,
             wp_prevLat, wp_prevDec, wp_prevRad) = net.receiveWaypoint()

            simulatedBoat.physicsModel().setActuators( delta_s, delta_r )

            # TODO - Jordan: Make this a variable step, so we aren't at a fixed rate of simulation
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
            (delta_r, delta_s, phi, latitude, longitude) = fcn.get_graph_values( simulatedBoat )

            threadLock.acquire()
            temp_data.set_value(x, y, theta, delta_s, delta_r, trueWind.direction(),
                                latitude, longitude, simulatedBoat.speed())
            temp_wp.set_value(wp_lon, wp_lat, wp_dec, wp_radius, wp_prevLon,
            wp_prevLat, wp_prevDec, wp_prevRad)
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
    temp_wp.set_run(0)
    threadLock.release()
    thread_draw.join()
