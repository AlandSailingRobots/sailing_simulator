class waypoint_handler(object):
    def __init__(self):
        self.lon = 0
        self.lat = 0
        self.dec = 0
        self.rad = 0
        self.prevlon = 0
        self.prevlat = 0
        self.prevdec = 0
        self.prevrad = 0
        self.run = 1

    def set_value(self, lon, lat, dec, rad, prevlon,
                  prevlat, prevdec, prevrad):
        self.lon = lon
        self.lat = lat
        self.dec = dec
        self.rad = rad
        self.prevlon = prevlon
        self.prevlat = prevlat
        self.prevdec = prevdec
        self.prevrad = prevrad

    def set_run(self, run_):
        self.run = run_

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
