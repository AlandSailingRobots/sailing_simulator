
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

from math import cos, sin, atan2, hypot

from matplotlib import pylab as plt
from matplotlib import lines

import json

import LatLonMath

temp_data = data_handler()
temp_wp = waypoint_handler()


def wrapTo2Pi(theta):
    if theta < 0:
        theta += 2*np.pi
    theta = theta % (2*np.pi)
    return theta


class drawThread (threading.Thread):
    def __init__(self, lock_):
        threading.Thread.__init__(self)
        self.lock = lock_
        self.run_th = 1
        self.threadID = 1
        self.name = "Draw thread"
        self.counter = 1

    def run(self):
        # global temp_wp, temp_data
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
        while(self.run_th):

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
            ax2.set_xlabel('Simulation of boat %0.1f %0.1f speed:%0.1f m/s rudder:%0.3f \
lat: %.5f lon: %.5f ' %
                           (wrapTo2Pi(-th_data.theta+np.pi/2)*180/np.pi,
                            wrapTo2Pi(-th_data.phi+np.pi/2)*180/np.pi,
                            th_data.speed,
                            th_data.delta_r,
                            th_data.latitude, th_data.longitude))

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
            # plt.show()
            plt.pause(0.001)
            lonprev = th_data.longitude
            latprev = th_data.latitude

        print("Stopping Draw Thread")
        plt.close()
