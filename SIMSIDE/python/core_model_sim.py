import numpy as np
from math import cos, sin, atan2, hypot
import time


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

    return np.array([x_dot, y_dot, theta_dot, v_dot, omega_dot])


class boat():
    def __init__(self, x_init=[0, 0, 0, 0, 0]):
        self.x = np.array(x_init)
        self.latitude = 0
        self.longitude = 0
        self.lat_origin = 0
        self.long_origin = 0
        self.course_real = 0
        self.course_magn = 0
        self.speed_knot = 0


if __name__ == '__main__':

    x = np.array([0, 0, 0, 0, 0], dtype='float64')

    while True:
        x += 0.5*f(x, 1, 1, 0.5, 0.1)
        print(x)
        time.sleep(0.1)
