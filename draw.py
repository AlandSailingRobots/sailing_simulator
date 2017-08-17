# Python standard packages
import threading
import copy
import queue
import time

# Our packages
import utils
import numpy as np
from utils import Functions as fcn
import core_draw_sim as cds

# Packages for plotting
from matplotlib.widgets import Button
from mpl_interaction import figure_pz
from matplotlib import pylab as plt
from matplotlib import lines
import matplotlib.patches as patches

# Memory profiler package
import objgraph


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


class drawThread (threading.Thread):
    def __init__(self, lock_, boat_type_, data_queue_, wp_queue_, ves_queue_):
        threading.Thread.__init__(self)
        self.lock = lock_
        self.run_th = 1
        self.threadID = 1
        self.name = "Draw thread"
        self.counter = 1
        self.boat_type = boat_type_
        self.data_queue = data_queue_
        self.wp_queue = wp_queue_
        self.ves_queue = ves_queue_

    def run(self):
        zoom_ = zoom()
        print(zoom_.length)
        axis_length = 0.008

        prevPos = []
        ais_list = self.ves_queue.get()
        for i in range(1, len(ais_list)):
            prevPos.append(ais_list[i].position())

        fig = figure_pz(figsize=(18, 9))
        fig.patch.set_facecolor('teal')
        fig.subplots_adjust(top=0.8)
        ax2 = fig.add_axes([0.45, 0.1, 0.35, 0.8])
        ax2.patch.set_facecolor('lightblue')

        axboat = fig.add_axes([0.05, 0.1, 0.35, 0.8])
        (axboat_min_x, axboat_min_y, axisboat_len) = (-20, -20, 40)
        axboat.patch.set_facecolor('lightblue')

        # th_data = copy.deepcopy(temp_data)
        th_data = self.data_queue.get()
        latprev = th_data.latitude
        lonprev = th_data.longitude

        # This is to draw from the starting point to the first waypoint
        prevWPlongitude = lonprev
        prevWPlatitude = latprev

        textbox = fig.text(0.8, 0.7, '')
        ax_bcenter = fig.add_axes([0.892, 0.33, 0.08, 0.05])

        # Button for center around the boat
        bcenter = Button(ax_bcenter, 'Center', color='white')
        bcenter.on_clicked(zoom_.boatInCenter)

        # Box that draws the wind direction
        windtext = fig.text(0.895, 0.51, 'Wind Direction')
        windtext.set_color('white')
        wind_ax = fig.add_axes([0.9,0.39,0.05,0.1])
        wind_ax.set_axis_off()
        cds.draw_wind_direction(wind_ax, (0, -0.5), 1, 0.3, th_data.phi)

        # Adds zoom functionality
        f = zoom_factory(ax2, zoom_)

        centerx = lonprev
        centery = latprev

        ax2.set_axis_off()
        print("Starting coordinates:", centerx, centery)

        axis_length = zoom_.length
        objgraph.show_most_common_types()
        (ax_min_x, ax_min_y, axis_len) = (centerx-axis_length/2, centery-axis_length/2, axis_length)
        while(self.run_th):
            if (ax_min_x-ax2.get_xlim()[0] != 0 or ax_min_y-ax2.get_ylim()[0] != 0) and ax2.get_xlim()[0] != 0:
                centerx = ax2.get_xlim()[0]+axis_length/2
                centery = ax2.get_ylim()[0]+axis_length/2
                zoom_.boatInCenter = False
            if zoom_.boatInCenter:
                centerx = th_data.longitude
                centery = th_data.latitude
            axis_length = zoom_.length

            th_data = self.data_queue.get()
            th_wp = self.wp_queue.get()
            ais_list = self.ves_queue.get()

            self.run_th = th_data.run

            (ax_min_x, ax_min_y, axis_len) = (centerx-axis_length/2, centery-axis_length/2, axis_length)

            btw = fcn.getBTW([th_data.longitude, th_data.latitude], [th_wp.lon, th_wp.lat])
            dtw = fcn.getDTW([th_data.longitude, th_data.latitude], [th_wp.lon, th_wp.lat])
            heading = utils.wrapTo2Pi(-th_data.theta+np.pi/2)*180/np.pi
            textstr = "ASV STATE\nLon:          %.5f\nLat:           %.5f\nHeading:  %.2f\nSpeed:       %.2f" % (th_data.longitude, th_data.latitude, heading, th_data.speed)
            textstr += "\n____________________\n\nRudder:   %.2f\nWingsail:  %.2f" % (th_data.delta_r, th_data.delta_s)
            textstr += "\n____________________\n\nWAYPOINT\nBearing:   %.2f\nDistance:  %.2f\nRadius:    %d" % (btw, dtw, th_wp.rad)

            fig.subplots_adjust(top=0.8)
            ax2 = fig.add_axes([0.45, 0.1, 0.4, 0.8])
            if th_wp.lon != prevWPlongitude:
                #  The following if statement needed due to on rare occasions
                #  we dont have any wp data to read in Network leading to
                #  us getting really small or super large values in th_wp
                if th_wp.lon > 0.01 and th_wp.lon < 1e3:
                    if prevWPlongitude != 0:
                        cds.draw_line(ax2, [th_wp.lon, th_wp.lat], [prevWPlongitude, prevWPlatitude], 'k')
                    cds.draw_wp(ax2, 1, th_wp.lon, th_wp.lat, th_wp.rad)
                    prevWPlongitude = th_wp.lon
                    prevWPlatitude = th_wp.lat

            dist = []
            minDist = 1e5
            for i in range(1, len(ais_list)):
                dist.append(fcn.getDTW([th_data.latitude, th_data.longitude], ais_list[i].position()))
                prevPos[i-1] = cds.draw_ais_track(ax2, ais_list[i].position(), prevPos[i-1], dist[i-1])
                cds.draw_ais(ax2, 0.0001, ais_list[i].position(), ais_list[i].course())
                minDist = min(minDist, dist[i-1])
            cds.draw_track(ax2, [th_data.longitude, th_data.latitude], [lonprev, latprev], minDist)
            cds.draw_boat(ax2, 0.00015, th_data.longitude, th_data.latitude,
                          th_data.theta, th_data.delta_r, th_data.delta_s)
            plt.axis([ax_min_x, ax_min_x+axis_len, ax_min_y, ax_min_y+axis_len])
            textstr += "\n____________________\n\nTRAFFIC\nDistance:  %.2f" % (minDist)

            #  Draws the vessel state (left side of the fig)
            axboat.clear()
            plt.sca(axboat)
            axboat_min_x = th_data.x-axisboat_len/2.0
            axboat_min_y = th_data.y-axisboat_len/2.0
            plt.axis([axboat_min_x, axboat_min_x+axisboat_len,
                      axboat_min_y, axboat_min_y+axisboat_len])
            if self.boat_type == 0:
                cds.draw_SailBoat(axboat, 1, th_data.x, th_data.y, th_data.theta, th_data.delta_r, th_data.delta_s)
            else:
                cds.draw_WingBoat(axboat, 1, th_data.x, th_data.y, th_data.theta, th_data.delta_r,th_data.MWAngle,th_data.delta_s)
            #  ---------------------------------------------

            plt.sca(ax2)
            ax2.patch.set_facecolor('lightblue')
            axboat.patch.set_facecolor('lightblue')
            fig.texts.remove(textbox)
            textbox = fig.text(0.9, 0.55, textstr,bbox=dict(edgecolor='white', facecolor='teal'))
            textbox.set_color('white')

            plt.pause(0.001)  # This is where the drawing happens

            lonprev = th_data.longitude
            latprev = th_data.latitude

            ax2.patches = []  # Clear out all the boats

            objgraph.show_growth(limit=5)  # Shows memory growth
            print()
        print("Stopping Draw Thread")
        plt.close()
