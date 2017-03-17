
import numpy as np
from math import cos, sin, atan2, hypot
import LatLongUTMconversion as LLUTM

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

class SailingBoat():
    def __init__(self, latitudeOrigin, longitudeOrigin):
        # Boat State
        self._latitude = latitudeOrigin
        self._longitude = longitudeOrigin
        self._courseReal = 0
        self._courseMagn = 0
        self._gpsSpeed = 0
        self._apparentWindSpeed = 0
        self._apparentWindDir = 0

        # Sim State
        self._x = 0
        self._y = 0
        self._heading = 0
        self._speed = 0
        self._rotationSpeed = 0
        self._rudderAngle = 0
        self._sailAngle = 0
        self._utmOriginX = 0
        self._utmOriginY = 0
        self._ref_ellipse = 23
        (self._utmZone, self._utmOriginX, self._utmOriginY) = LLUTM.LLtoUTM( self._ref_ellipse, self._latitude, self._longitude)

    def setSail( self, sailAngle ):
        self._sailAngle = sailAngle
    
    def setRudder( self, rudderAngle ):
        self._rudderAngle = rudderAngle
    
    # Returns the boats sail angle
    def sail(self):
        return self._sailAngle

    # Returns the rudder angle
    def rudder(self):
        return self._rudderAngle

    def latitude(self):
        return self._latitude

    def longitude(self):
        return self._longitude
        # GPS course
    def courseReal(self):
        return self._courseReal
        # Compass heading
    def courseMagn(self):
        return self._courseMagn

    def gpsSpeed(self):
        return self._gpsSpeed

    def apparentWindDirection(self):
        return self._apparentWindDir

    def apparentWindSpeed(self):
        return self._apparentWindSpeed

    def simulate(self, timestep, trueWindSpeed, trueWindAngle):
        (x,
         apparentWindSpeed,
         apparentWindAngle,
         speed) = self.one_loop(timestep, trueWindSpeed, trueWindAngle)

        (self._latitude, self._longitude) = LLUTM.UTMtoLL(self._ref_ellipse, self._utmOriginY+x[1], self._utmOriginX+x[0], self._utmZone)

        self._courseReal = wrapTo2Pi( -atan2( speed[1], speed[0] ) + np.pi / 2 ) * 180 / np.pi
        self._courseMagn = wrapTo2Pi( -self._heading + np.pi / 2 ) * 180/ np.pi

        self._gpsSpeed = hypot(speed[0], speed[1])
        self._apparentWindDir = wrapTo2Pi( -apparentWindAngle + np.pi ) * 180 / np.pi
        self._apparentWindSpeed = apparentWindSpeed

        return (x, apparentWindSpeed, apparentWindAngle, speed)

    def one_loop(self, dt, trueWindSpeed, trueWindAngle):
        (df, a_ap, phi_ap, speed) = f(np.array([self._x, self._y, self._heading, self._speed, self._rotationSpeed]), trueWindSpeed, trueWindAngle, self._sailAngle, self._rudderAngle)
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