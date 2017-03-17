import numpy as np
from math import cos, sin, atan2, hypot
import time
import LatLongUTMconversion as LLUTM

from sailing_boat import SailingBoat


# Calculate the drift of and change in position from the boat's current velocity and the force of
# the wind on the sails
def calculateDrift( driftCoefficient, boatSpeed, boatHeading, trueWindSpeed, trueWindAngle ):
    x_boatVelocity = boatSpeed * cos(boatHeading)
    y_boatVelocity = boatSpeed * sin(boatHeading)

    x_windDrift = driftCoefficient * trueWindSpeed * cos(trueWindAngle)
    y_windDrift = driftCoefficient * trueWindSpeed * sin(trueWindAngle)

    x_dot = x_boatVelocity + x_windDrift
    y_dot = y_boatVelocity + y_windDrift

    return (x_dot, y_dot)

def calculateApparentWind( trueWindAngle, trueWindSpeed, boatHeading, boatSpeed ):
    apparentWindVector = [trueWindSpeed * cos( trueWindAngle - boatHeading ) - boatSpeed, trueWindSpeed * sin(trueWindAngle - boatHeading)]

    apparentWindAngle = atan2(apparentWindVector[1], apparentWindVector[0])
    apparentWindSpeed = hypot(apparentWindVector[0], apparentWindVector[1]) 

    return ( apparentWindAngle, apparentWindSpeed )

def calculateCorrectSailAngle( apparentWindAngle, sailAngle ):
    sigma = cos(apparentWindAngle) + cos(sailAngle)
    if (sigma < 0):
        return np.pi + apparentWindAngle
    elif sin(apparentWindAngle) is not 0:
        # Ensure the sail is on the right side of the boat
        return -np.sign( sin( apparentWindAngle ) ) * abs(sailAngle)
    else:
        return sailAngle

def f(currState, trueWindSpeed, trueWindAngle, sailAngle, rudderAngle):
    driftCoefficient = 0.03
    tangentialFriction = 40     # kg s^-1
    angularFriction = 6000      # kg m
    sailLift = 500              # kg s^-1
    rudderLift = 2000           # kg s^-1
    distanceToSailCoE = 0.5     # m
    distanceToMast = 0.5        # m
    distanceToRudder = 2        # m
    boatMass = 300              # kg
    momentOfInertia = 400       # kg m^2
    rudderBreakCoefficient = 0.2         

    boatHeading = currState[2]
    speed = currState[3]
    rotationSpeed = currState[4]

    (x_dot, y_dot) = calculateDrift( driftCoefficient, speed, boatHeading, trueWindSpeed, trueWindAngle)
    (apparentWindAngle, apparentWindSpeed) = calculateApparentWind( trueWindAngle, trueWindSpeed, boatHeading, speed )
    sailAngle = calculateCorrectSailAngle( apparentWindAngle, sailAngle )

    # The acceleration of the boat is affected by three forces, the wind on the sail, a braking force
    # from the water on the rudder, and a tangential friction force
    forceOnSail = sailLift * apparentWindSpeed * sin( sailAngle - apparentWindAngle )
    forceOnRudder = rudderLift * speed * sin( rudderAngle )

    sailAccelerationForce = forceOnSail * sin( sailAngle )
    rudderBrakeForce = rudderBreakCoefficient * forceOnRudder * sin( sailAngle )
    tangentialFictionForce = np.sign(speed) * (tangentialFriction * (speed)**2)

    acceleration_dot = ( (sailAccelerationForce - rudderBrakeForce) - tangentialFictionForce) / boatMass

    sailRotationForce = forceOnSail * ( distanceToSailCoE - distanceToMast * cos(sailAngle) )
    rudderRotationForce = distanceToRudder * forceOnRudder * cos(rudderAngle)
    rotationForce = angularFriction * rotationSpeed * abs(speed)

    rotationSpeed_dot = (sailRotationForce - rudderRotationForce - rotationForce) / momentOfInertia

    return (np.array([x_dot, y_dot, rotationSpeed, acceleration_dot, rotationSpeed_dot]),
            apparentWindSpeed,
            apparentWindAngle,
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
    def __init__(self):
        self._x = 0
        self._y = 0
        self._heading = 0
        self._speed = 0
        self._rotationSpeed = 0

    def one_loop(self, dt, a, phi, delta_s, delta_r):
        (df, a_ap, phi_ap, speed) = f(np.array([self._x, self._y, self._heading, self._speed, self._rotationSpeed]), a, phi, delta_s, delta_r)
        #self.x += dt*df
        self._x += df[0] * dt
        self._y += df[1] * dt
        self._heading += df[2] * dt
        self._speed += df[3] * dt
        self._rotationSpeed += df[4] * dt

        self._heading = wrapTo2Pi(self._heading)
        return (np.array([self._x, self._y, self._heading, self._speed, self._rotationSpeed]), a_ap, phi_ap, speed)

    def get_graph_values(self):
        return (self._x, self._y, self._heading)


class simulation(object):
    def __init__(self, ref_ellipse=23,
                 lat_origin=60.107240,
                 long_origin=19.922397,
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
        self.speed_knot = 0
        self.boat_ = SailingBoat(lat_origin, long_origin)
        self.a = a
        self.phi = phi
        self.pressure = 181
        self.rudder = 0
        self.sheet = 0
        self.battery = 0
        self.apparentWindSpeed = 0
        self.phi_ap = 0
        self.apparentWindSpeed_ = 0
        self.heading = 0

    def set_actuators(self, command_rudder, command_sheet):
        (delta_r, delta_s) = order_to_deg(command_rudder,
                                                    command_sheet)
        self.boat_.setSail(delta_s)
        self.boat_.setRudder(delta_r)

    def set_wind(self, windspeed, wind_direction):
        self.a = windspeed
        self.phi = wind_direction

    def one_loop(self, dt):
        (x,
         self.apparentWindSpeed_,
         self.phi_ap_,
         speed) = self.boat_.simulate(dt, self.a, self.phi)

        self.apparentWindSpeed = self.apparentWindSpeed_                      # kept in m/s
        self.phi_ap = wrapTo2Pi(-self.phi_ap_+np.pi)*180/np.pi
        self.heading = self.boat_.courseMagn()

    def get_to_socket_value(self):
        heading = ((self.boat_.courseMagn(), 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0, 0))
        gps = (self.boat_.latitude(), self.boat_.longitude(), self.boat_.courseReal(),
               self.boat_.courseMagn(), self.boat_.gpsSpeed())
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
                self.phi_ap_, self.phi, self.boat_.latitude(), self.boat_.longitude())

if __name__ == '__main__':

    x = np.array([0, 0, 0, 0, 0], dtype='float64')

    while True:
        x += 0.5*f(x, 1, 1, 0.5, 0.1)
        print(x)
        time.sleep(0.1)
