import numpy as np
from math import cos, sin, atan2, hypot
import time
import LatLongUTMconversion as LLUTM


def f(y, a, phi, delta_s, delta_r):
    theta = y[2]
    v = y[3]
    omega = y[4]

    # Internal constants
    p1 = 0.03         # -         drift coefficientv_norm
    p2 = 40           # kg s^-1    tangential friction
    p3 = 6000         # kg m       angular friction
    p4 = 500          # kg s^-1    sail lift
    p5 = 2000         # kg s^-1    rudder lift
    p6 = 0.5          # m          distance to sail CoE
    p7 = 0.5          # m          distance to mast
    p8 = 2            # m          distance to rudder
    p9 = 300          # kg         mass of boat
    p10 = 400         # kg m^2     mass moment of intertia
    p11 = 0.2         # rudder break coefficient

    # link equations
    W_ap = [a*cos(phi-theta)-v, a*sin(phi-theta)]

    # apparent wind speed vector in b-frame
    phi_ap = atan2(W_ap[1], W_ap[0])   # apparent wind angle in b-frame
    a_ap = hypot(W_ap[0], W_ap[1])      # apparent wind speed velocity

    sigma = cos(phi_ap)+cos(delta_s)
    if (sigma < 0):
        delta_s = np.pi+phi_ap
    else:
        if sin(phi_ap)is not 0:
            delta_s = -np.sign(sin(phi_ap))*abs(delta_s)

    Fs = p4*a_ap*sin(delta_s-phi_ap)  # Force of wind on sail
    Fr = p5*v*sin(delta_r)            # Force of water on rudder

    x_dot = v*cos(theta) + p1*a*cos(phi)     # x_dot
    y_dot = v*sin(theta) + p1*a*sin(phi)   # y_dot
    v_dot = ((Fs*sin(delta_s)-p11*Fr*sin(delta_r))-np.sign(v)*(p2*(v)**2))/p9
    theta_dot = omega                # theta_dot
    omega_dot = (Fs*(p6-p7*cos(delta_s))-p8*Fr*cos(delta_r) - p3*omega*abs(v)
                 )/p10

    return (np.array([x_dot, y_dot, theta_dot, v_dot, omega_dot]),
            a_ap,
            phi_ap,
            [x_dot, y_dot])


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


class Boat():
    def __init__(self, x_init=[0, 0, 0, 0, 0]):
        self.x = np.array(x_init, dtype='float64')

    def one_loop(self, dt, a, phi, delta_s, delta_r):
        (df, a_ap, phi_ap, speed) = f(self.x, a, phi, delta_s, delta_r)
        self.x += dt*df
        self.x[2] = wrapTo2Pi(self.x[2])
        return (self.x, a_ap, phi_ap, speed)

    def get_graph_values(self):
        return (self.x[0], self.x[1], self.x[2])


class simulation(object):
    def __init__(self, ref_ellipse=23,
                 lat_origin=60.073090,
                 long_origin=19.8985,
                 x_init=[0, 0, np.pi/2+0.3, 0, 0],
                 dt=0.1,
                 a=2,
                 phi=1):

        self.latitude = 0
        self.longitude = 0
        self.ref_ellipse = ref_ellipse
        self.lat_origin = lat_origin
        self.long_origin = long_origin
        (self.UTMZone, self.utm_x_origin, self.utm_y_origin) = LLUTM.LLtoUTM(
                                          self.ref_ellipse,
                                          self.lat_origin,
                                          self.long_origin)
        self.course_real = 0
        self.course_magn = 0
        self.speed_knot = 0
        self.boat_ = Boat(x_init)
        self.a = a
        self.phi = phi
        self.pressure = 181
        self.delta_s = 0
        self.delta_r = 0
        self.rudder = 0
        self.sheet = 0
        self.battery = 0
        self.a_ap = 0
        self.phi_ap = 0
        self.a_ap_ = 0
        self.phi_ap_ = 0
        self.heading = 0

    def set_actuators(self, command_rudder, command_sheet):
        (self.delta_r, self.delta_s) = order_to_deg(command_rudder,
                                                    command_sheet)

    def set_wind(self, windspeed, wind_direction):
        self.a = windspeed
        self.phi = wind_direction

    def one_loop(self, dt):
        (x,
         self.a_ap_,
         self.phi_ap_,
         speed) = self.boat_.one_loop(dt, self.a,
                                      self.phi,
                                      self.delta_s,
                                      self.delta_r)
        (self.latitude, self.longitude) = LLUTM.UTMtoLL(
                        self.ref_ellipse,
                        self.utm_y_origin+x[1],
                        self.utm_x_origin+x[0],
                        self.UTMZone)
        self.course_real = wrapTo2Pi(-atan2(speed[1],
                                     speed[0])+np.pi/2)*180/np.pi
        self.course_magn = wrapTo2Pi(-x[2]+np.pi/2)*180/np.pi
        self.rudder = int(rudder_deg_to_arduino(self.delta_r))
        self.speed_knot = hypot(speed[0], speed[1])*1.94384  # m/s to knot
        self.a_ap = self.a_ap_                      # kept in m/s
        self.phi_ap = wrapTo2Pi(-self.phi_ap_+np.pi)*180/np.pi
        print("send phi_ap: ", self.phi_ap, " from ", phi_ap_*180/np.pi)
        self.heading = self.course_magn

    def get_to_socket_value(self):
        heading = ((self.heading, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0, 0))
        gps = (self.latitude, self.longitude, self.course_real,
               self.course_magn, self.speed_knot)
        arduino = (self.pressure, self.rudder, self.sheet, self.battery)
        windsensor = (self.a_ap, self.phi_ap)
        return (heading, gps, arduino, windsensor)

    def get_boat(self):
        return self.boat_

    def get_graph_values(self):
        delta_s_g = self.delta_s
        sigma = cos(self.phi_ap_)+cos(self.delta_s)
        if (sigma < 0):
            delta_s_g = np.pi+self.phi_ap_
        else:
            if sin(self.phi_ap_)is not 0:
                delta_s_g = -np.sign(sin(self.phi_ap_))*abs(self.delta_s)
        return (self.delta_r, delta_s_g,
                self.phi_ap_, self.phi, self.latitude, self.longitude)

if __name__ == '__main__':

    x = np.array([0, 0, 0, 0, 0], dtype='float64')

    while True:
        x += 0.5*f(x, 1, 1, 0.5, 0.1)
        print(x)
        time.sleep(0.1)
