#!/usr/bin/python

"""
An attempt at optimizing using pyqtgraph instead of matplotlib that is supposedly faster
Not a big improvement this far, the big problem seems to be that the main process takes too long to put data in the queue
And also one limiting factor is still all the small line segments we add when drawing the track
As of right now, we draw the track of our boat and the ais traffic
"""
# Command line arguments:
# boat_type: 0 or 1
# 0, simulating with Janet
# 1, simulatiing with ASPire
# traffic: 0 or 1
# 0, simulating without traffic
# 1, simulating with traffic
# Example: simulation_main.py 0 1 (simulate Janet with traffic)

import memory_profiler
import socket
import sys
import select
import threading
import time
import copy
import atexit
from struct import *
from utils import wrapTo2Pi,wrapAngle,radTodeg, degTorad
import numpy as np
import core_draw_sim as cds
from simulator import Simulator
from physics_models import SimplePhysicsModel,SailingPhysicsModel, WindState, ASPirePhysicsModel
from vessel import Vessel,SailBoat, MarineTraffic
from network import Network
from utils import Functions as fcn
from data_handler import data_handler, waypoint_handler, sailBoatData, wingBoatData
import multiprocessing as mp
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg
# from pyqtgraph.ptime import time


# from drawThread import drawThread
from math import cos, sin, atan2, hypot
from matplotlib.widgets import Button
from mpl_interaction import figure_pz
import objgraph
from matplotlib.collections import LineCollection

from matplotlib import pylab as plt
from matplotlib import lines
import matplotlib.patches as patches

import json

import LatLonMath

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
        threadLock.release()
        # thread_draw.join()


atexit.register(exit_function_py)
init_prog = 0


BOAT_UPDATE_MS = 100
AIS_UPDATE_MS = 500
CAMERA_ANGLE = 24
boatInCenter = False


def wrapTo2Pi(theta):
    if theta < 0:
        theta += 2*np.pi
    theta = theta % (2*np.pi)
    return theta


def zoom_factory(ax, zoom, base_scale=0.99):
    def zoom_fun(event):
        # get the current x and y limits
        cur_xlim = ax.get_xlim()
        cur_ylim = ax.get_ylim()
        cur_xrange = (cur_xlim[1] - cur_xlim[0])*.5
        cur_yrange = (cur_ylim[1] - cur_ylim[0])*.5
        xdata = event.xdata  # get event x location
        ydata = event.ydata  # get event y location
        if event.button == 'up':
            # deal with zoom in
            if zoom.length > 0.001:
                zoom.length -= 0.001
            scale_factor = 1/base_scale
        elif event.button == 'down':
            # deal with zoom out
            if zoom.length < 0.15:
                zoom.length += 0.001
            scale_factor = base_scale
        else:
            # deal with something that should never happen
            scale_factor = 1
            print (event.button)

    fig = ax.get_figure()  # get the figure of interest
    # attach the call back
    fig.canvas.mpl_connect('scroll_event',zoom_fun)

    # return the function
    return zoom_fun


class zoom(object):
    length = 0.008
    boatInCenter = True

    def boatInCenter(self, event):
        self.boatInCenter = True


def draw_boastate(ax, boat_type, q):
    while True:
        print("HEJ")
        # axboat = fig.add_axes([0.05, 0.1, 0.35, 0.8])
        (axboat_min_x, axboat_min_y, axisboat_len) = (-20, -20, 40)
        ax.patch.set_facecolor('lightblue')
        # th_data = copy.deepcopy(temp_data)
        # while True:
        # th_data = copy.deepcopy(temp_data)
        th_data = q.get()
        ax.clear()
        axboat_min_x = th_data.x-axisboat_len/2.0
        axboat_min_y = th_data.y-axisboat_len/2.0
        plt.axis([axboat_min_x, axboat_min_x+axisboat_len,
                  axboat_min_y, axboat_min_y+axisboat_len])

        print("x:",th_data.x)
        if boat_type == 0:
            cds.draw_SailBoat(ax, 1, th_data.x, th_data.y, th_data.theta, th_data.delta_r, th_data.delta_s)
        else:
            cds.draw_WingBoat(ax, 1, th_data.x, th_data.y, th_data.theta, th_data.delta_r,th_data.MWAngle,th_data.delta_s)
        plt.pause(0.1)

def get_graph_values( sailBoat,boat_type ):
	(sail, rudder) = sailBoat.sailAndRudder() # if boat_type == 1 : sail == tailWing
	phi_ap = sailBoat.apparentWind().direction()
	(lat, lon) = sailBoat.position()
	if boat_type == 1:
		MWAngle   = sailBoat.physicsModel().MWAngle()
		tailAngle = sail
		return( rudder, tailAngle, phi_ap, lat, lon, MWAngle )
	else:

		sigma = cos( phi_ap ) + cos( sail )
		if (sigma < 0):
			sail = np.pi - phi_ap
		else:
			sail = np.sign( sin(phi_ap) ) * abs( sail )
		return( rudder, sail, phi_ap, lat, lon )


def loadConfiguration(configPath, traffic, boat_type):
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
    print(latOrigin, lonOrigin)
    # vessels.append(SailBoat( SailingPhysicsModel(), latOrigin, lonOrigin, 0, 0 ))
    if boat_type == 0:
        vessels.append(SailBoat( SailingPhysicsModel(), latOrigin, lonOrigin, 0, 0 ))
    else:
        # print(trueWindDir)
        # mainBoat = SailBoat( ASPirePhysicsModel( 0,0,0,wrapTo2Pi(trueWindDir)) , latOrigin, lonOrigin, 0, 0 )
        # print('MWAnglePhysicsClass:',mainBoat.physicsModel().MWAngle())
        vessels.append(SailBoat( ASPirePhysicsModel( 0,0,0,wrapTo2Pi(trueWindDir+degTorad(185))), latOrigin, lonOrigin, 0, 0 ))

    # Load Marine Traffic
    if traffic == 1:
        for marineVessel in config["traffic"]:
            id = marineVessel["mmsi"]
            lat = marineVessel["lat_origin"]
            lon = marineVessel["lon_origin"]
            heading = wrapTo2Pi(np.deg2rad(marineVessel["heading"] - 90))
            speed = marineVessel["speed"]
            length = marineVessel["length"]
            beam = marineVessel["beam"]
            vessels.append(MarineTraffic(SimplePhysicsModel(heading, speed), lat, lon, heading, speed, id, length, beam))

    return ( boat_type, vessels, WindState( trueWindDir, trueWindSpeed ) )


# temp_data = data_handler()
temp_wp = waypoint_handler()


def display(q, wq, vesq):
    app = QtGui.QApplication([])

    track_x = []
    track_y = []
    print("HEJ")
    win = pg.GraphicsWindow(title="Basic plotting examples")
    # pg.plot(track_x)
    # p = pg.plot(track_x)
    # p1 = win.addPlot(title="Test")
    p2 = win.addPlot(title="Test")
    # win.setWindowTitle('Simulator')
    count = 0
    th_data = q.get()
    ais = vesq.get()

    t_x = []
    t_y = []
    ais_x = []
    ais_y = []
    for i in range(1,len(ais)):
        ais_x.append([])
        ais_y.append([])

    def update():  # p2,q,track_x,track_y):
        global p, track_x, track_y, th_data
        # print("HEJ")
        th_data = q.get()
        ais = vesq.get()
        if len(track_x) < 2:
            track_x.insert(0,th_data.longitude)
            track_y.insert(0,th_data.latitude)
            track_x.insert(0,th_data.longitude)
            track_y.insert(0,th_data.latitude)
        track_x.insert(0,th_data.longitude)
        track_y.insert(0,th_data.latitude)
        for i in range(1,len(ais)):
            if len(ais_x[i-1]) == 0:
                ais_x[i-1].insert(0,ais[i].position()[1])
                ais_y[i-1].insert(0,ais[i].position()[0])
            ais_x[i-1].insert(0,ais[i].position()[1])
            ais_y[i-1].insert(0,ais[i].position()[0])
            # p2.plot(ais_x[i-1], ais_y[i-1])
            p2.plot([ais_x[i-1][0], ais_x[i-1][1]], [ais_y[i-1][0], ais_y[i-1][1]],pen='y')
        # p2.plot(track_x,track_y)
        t_x.append(track_x[0])
        t_x.append(track_x[1])
        t_y.append(track_y[0])
        t_y.append(track_y[1])
        p2.plot(t_x, t_y, pen='m')
        p2.setTitle('Count: %d' % count)

    timer = QtCore.QTimer()
    timer.timeout.connect(update)  # p2,q,t_x,t_y))
    timer.start()
    QtGui.QApplication.instance().exec_()


if __name__ == '__main__':
    # global temp_wp, temp_data
    net = Network( "localhost", 6900 )

    configPath = "config.json"

    ais_vessels = []
    track_x = []
    track_y = []

    dq = mp.Queue()
    wq = mp.Queue()
    vesq = mp.Queue()
    # if len(sys.argv) == 2:
    #     configPath = sys.argv[1]

    if len(sys.argv) == 3:
        boat_type = int(sys.argv[1])
        traffic = int(sys.argv[2])
    else:
        boat_type = 0
        traffic = 1

    ( boat_type, vessels, trueWind ) = loadConfiguration(configPath, traffic, boat_type)

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

    fig = figure_pz(figsize=(18, 9))
    axboat = fig.add_axes([0.05, 0.1, 0.35, 0.8])
    # multithreading management:
    threadLock = threading.Lock()
    # thread_draw = drawThread(threadLock, fig)
    init_prog = 1

    dt = 0.1

    (x, y) = simulatedBoat.physicsModel().utmCoordinate()
    theta = simulatedBoat.physicsModel().heading()
    (delta_r, delta_s, phi, latitude, longitude) = get_graph_values(simulatedBoat, boat_type)
    temp_data.set_value(x, y, theta, delta_s, delta_r, trueWind.direction(),latitude, longitude, simulatedBoat.speed())

    bytes_received = 0
    data = bytearray()

    time.sleep(0.05)
    delta_t = 0.05

    print("Start drawing thread")
    # thread_draw.start()
    # man = mp.Manager()
    # que = man.Queue()
    p = mp.Process(target=display, args=(dq, wq, vesq))
    p.start()
    # pool = mp.Pool()

    # pool.apply_async(draw_boastate, (axboat, boat_type, que))

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
                (delta_r, delta_s, phi, latitude, longitude) = get_graph_values(simulatedBoat, boat_type)
            else:
                (delta_r, delta_s, phi, latitude, longitude, MWAngle) = get_graph_values(simulatedBoat, boat_type)
            track_x.append(longitude)
            track_y.append(latitude)
            # threadLock.acquire()
            if boat_type == 0:
                temp_data.set_value(x, y, theta, delta_s, delta_r, trueWind.direction(),latitude, longitude, simulatedBoat.speed())
            else:
                temp_data.set_value(x, y, theta, delta_s, delta_r, trueWind.direction(),latitude, longitude, simulatedBoat.speed(), MWAngle)
            dq.put(temp_data)

            # print(temp_data.longitude, temp_data.latitude)
            temp_wp.set_value(wp_lon, wp_lat, wp_dec, wp_radius, wp_prevLon,
                              wp_prevLat, wp_prevDec, wp_prevRad)
            # print("Main x:", temp_data.x)
            wq.put(temp_wp)
            vesq.put(vessels)
            # threadLock.release()

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
    # p.join()
    threadLock.acquire()
    temp_data.set_run(0)
    temp_wp.set_run(0)
    threadLock.release()

    p.join()
    # thread_draw.join()
