
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
        # set new limits
        # ax.set_xlim([xdata - cur_xrange*scale_factor# xdata + cur_xrange*scale_factor])
        #  ax.set_ylim([ydata - cur_yrange*scale_factor,# ydata + cur_yrange*scale_factor])
        #  plt.draw()  # force re-draw

    fig = ax.get_figure()  # get the figure of interest
    # attach the call back
    fig.canvas.mpl_connect('scroll_event',zoom_fun)

    # return the function
    return zoom_fun


class zoom(object):
    length = 0.008
    boatInCenter = True

    def zoomout(self, event):
        if self.length < 0.04:
            self.length += 0.001
        print(self.length)

    def zoomin(self, event):
        if self.length > 0.005:
            self.length -= 0.001
        print(self.length)

    def boatInCenter(self, event):
        self.boatInCenter = True


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

        xlist = []
        ylist = []
        linecol = []

        fig = figure_pz(figsize=(9, 9))
        fig.patch.set_facecolor('teal')
        fig.subplots_adjust(top=0.8)
        ax2 = fig.add_axes([0.1, 0.1, 0.7, 0.8])
        ax2.patch.set_facecolor('lightblue')

        th_data = copy.deepcopy(temp_data)
        latprev = th_data.latitude
        lonprev = th_data.longitude
        prevWPlongitude = lonprev
        prevWPlatitude = latprev

        textbox = fig.text(0.7, 0.7, '')
        ax_bcenter = fig.add_axes([0.812, 0.43, 0.08, 0.05])

        bcenter = Button(ax_bcenter, 'Center', color='white')
        bcenter.on_clicked(zoom_.boatInCenter)
        windtext = fig.text(0.815, 0.6, 'Wind Direction')
        windtext.set_color('white')
        wind_ax = fig.add_axes([0.82,0.49,0.1,0.1])
        wind_ax.set_axis_off()
        cds.draw_wind_direction(wind_ax, (0, -0.5), 1, 0.3, th_data.phi)

        f = zoom_factory(ax2, zoom_)

        centerx = lonprev
        centery = latprev
        print(centerx, centery)
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
            self.lock.acquire()
            th_data = copy.deepcopy(temp_data)
            th_wp = copy.deepcopy(temp_wp)
            ais_list = copy.deepcopy(vessels)
            self.lock.release()
            self.run_th = th_data.run
            (ax_min_x, ax_min_y, axis_len) = (centerx-axis_length/2, centery-axis_length/2, axis_length)

            btw = fcn.getBTW([th_data.longitude, th_data.latitude], [th_wp.lon, th_wp.lat])
            dtw = fcn.getDTW([th_data.longitude, th_data.latitude], [th_wp.lon, th_wp.lat])
            heading = wrapTo2Pi(-th_data.theta+np.pi/2)*180/np.pi
            textstr = "ASV STATE\nLon:          %.5f\nLat:           %.5f\nHeading:  %.2f\nSpeed:       %.2f" % (th_data.longitude, th_data.latitude, heading, th_data.speed)
            textstr += "\n____________________\n\nRudder:   %.2f\nWingsail:  %.2f" % (th_data.delta_r, th_data.delta_s)
            textstr += "\n____________________\n\nWAYPOINT\nBearing:   %.2f\nDistance:  %.2f\nRadius:    %d" % (btw, dtw, th_wp.rad)

            # ax2.clear()
            # plt.cla()   # Clear axis
            # plt.clf()   # Clear figure

            fig.subplots_adjust(top=0.8)
            ax2 = fig.add_axes([0.1, 0.1, 0.7, 0.8])

            if th_wp.lon != prevWPlongitude:
                if th_wp.lon > 0.01 and th_wp.lon < 1e3:
                    if prevWPlongitude != 0:
                        cds.draw_line(ax2, [th_wp.lon, th_wp.lat], [prevWPlongitude, prevWPlatitude], 'k')
                    cds.draw_wp(ax2, 1, th_wp.lon, th_wp.lat, th_wp.rad)
                    cds.draw_wp(ax2, 1, th_wp.lon, th_wp.lat, th_wp.rad)
                    prevWPlongitude = th_wp.lon
                    prevWPlatitude = th_wp.lat
            # cds.draw_track(ax2, [th_data.longitude, th_data.latitude], [lonprev, latprev])

            # xlist.append(th_data.longitude)
            linecol.append((th_data.longitude,th_data.latitude))
            print(linecol)
            # xlist.append(None)
            # ylist.append(th_data.latitude)
            # ylist.append(None)

            # for i in range(1, len(ais_list)):
            #     cds.draw_track(ax2, ais_list[i].position(), ais_list[i].course())
            for i in range(1, len(ais_list)):
                cds.draw_ais(ax2, 0.0001, ais_list[i].position(), ais_list[i].course())
            cds.draw_boat(ax2, 0.00015, th_data.longitude, th_data.latitude,
                          th_data.theta, th_data.delta_r, th_data.delta_s)
            plt.axis([ax_min_x, ax_min_x+axis_len, ax_min_y, ax_min_y+axis_len])
            # ax2.plot(xlist,ylist,'r-',alpha=0.5)
            linessss = LineCollection(linecol,alpha=0.1)
            print(linessss)
            # ax2.add_collection(linessss)
            plt.draw()
            fig.texts.remove(textbox)
            textbox = fig.text(0.82, 0.63, textstr,bbox=dict(edgecolor='white', facecolor='teal'))
            textbox.set_color('white')
            # print(ax2.get_children())
            # plt.show()
            plt.pause(0.001)
            lonprev = th_data.longitude
            latprev = th_data.latitude
            ax2.patches = []
            # lines = []
            objgraph.show_most_common_types()
            print()
        print("Stopping Draw Thread")
        plt.close()
