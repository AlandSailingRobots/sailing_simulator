import numpy as np
from math import cos, sin, atan2, hypot
import time
import LatLongUTMconversion as LLUTM

from sailing_boat import SailingBoat


def wrapTo2Pi(theta):
    if theta < 0:
        theta += 2*np.pi
    theta = theta % (2*np.pi)
    return theta

def rudder_deg_to_arduino(delta):
    return int(1500*delta*(6/np.pi)*(235/1500.0)+285)

def order_to_deg(command_rudder, command_sheet):
    if command_rudder > 8000 or command_rudder < 3000:
        command_sheet = 4215
        command_rudder = 5520
    return ((command_rudder-5520)*(np.pi/6.0)/1500.0,
            (command_sheet-4215)*(np.pi/-6.165)/900.0)

class WindState(object):
    def __init__(self, windDirection, windSpeed):
        self._dir = windDirection
        self._spd = windSpeed

    def direction(self):
        return self._dir

    def speed(self):
        return self._spd

class simulation(object):
    def __init__(self, simulatedBoat, trueWind,
                 dt=0.1,
                 a=2,
                 phi=1):
        self.boat_ = simulatedBoat
        self.pressure = 181
        self.phi_ap = 0
        self._trueWind = trueWind

    def set_actuators(self, command_rudder, command_sheet):
        (delta_r, delta_s) = order_to_deg(command_rudder,
                                                    command_sheet)
        self.boat_.setSail(delta_s)
        self.boat_.setRudder(delta_r)

    def one_loop(self, dt):
        ( self.phi_ap_ ) = self.boat_.simulate(dt, self._trueWind )
                  # kept in m/s
        self.phi_ap = wrapTo2Pi(-self.phi_ap_+np.pi)*180/np.pi

    def get_to_socket_value(self):
        heading = ((self.boat_.compassHeading(), 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0, 0))
        gps = (self.boat_.latitude(), self.boat_.longitude(), self.boat_.gpsCourse(),
               self.boat_.compassHeading(), self.boat_.gpsSpeed())
        arduino = (0, 0, 0, 0)
        windsensor = (self.boat_.apparentWindSpeed(), self.boat_.apparentWindDirection())
        return (heading, gps, arduino, windsensor)

    def get_boat(self):
        return self.boat_

    def get_graph_values(self):
        delta_s_g = self.boat_.sail()
        sigma = cos(self.phi_ap_)+cos(self.boat_.sail())
        if (sigma < 0):
            delta_s_g = np.pi+self.phi_ap_
        else:
            if sin(self.phi_ap_)is not 0:
                delta_s_g = -np.sign(sin(self.phi_ap_))*abs(self.boat_.sail())
        return (self.boat_.rudder(), delta_s_g,
                self.phi_ap_, self._trueWind.direction(), self.boat_.latitude(), self.boat_.longitude())
