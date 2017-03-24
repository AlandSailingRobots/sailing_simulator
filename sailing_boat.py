
import numpy as np
from math import cos, sin, atan2, hypot
import LatLongUTMconversion as LLUTM
import core_model_sim as cms
from physics_models import SailingPhysicsModel, WindState

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

def calculateApparentWind( trueWind, boatHeading, boatSpeed ):
    apparentWindVector = [trueWind.speed() * cos( trueWind.direction() - boatHeading ) - boatSpeed, trueWind.speed() * sin(trueWind.direction() - boatHeading)]

    apparentWindAngle = atan2(apparentWindVector[1], apparentWindVector[0])
    apparentWindSpeed = hypot(apparentWindVector[0], apparentWindVector[1]) 

    return cms.WindState( apparentWindAngle, apparentWindSpeed )

def calculateCorrectSailAngle( apparentWind, sailAngle ):
    sigma = cos(apparentWind.direction()) + cos(sailAngle)
    if (sigma < 0):
        return np.pi + apparentWind.direction()
    elif sin(apparentWind.direction()) is not 0:
        # Ensure the sail is on the right side of the boat
        return -np.sign( sin( apparentWind.direction() ) ) * abs(sailAngle)
    else:
        return sailAngle

def f(currState, trueWind, sailAngle, rudderAngle):
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

    (x_dot, y_dot) = calculateDrift( driftCoefficient, speed, boatHeading, trueWind.speed(), trueWind.direction() )
    apparentWind = calculateApparentWind( trueWind, boatHeading, speed )
    sailAngle = calculateCorrectSailAngle( apparentWind, sailAngle )

    # The acceleration of the boat is affected by three forces, the wind on the sail, a braking force
    # from the water on the rudder, and a tangential friction force
    forceOnSail = sailLift * apparentWind.speed() * sin( sailAngle - apparentWind.direction() )
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
            apparentWind,
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
        self._physics = SailingPhysicsModel()

    # Boat State In terms of the control system
    def latitude(self):
        return self._latitude

    def longitude(self):
        return self._longitude

    def gpsCourse(self):
        return self._courseReal

    def compassHeading(self):
        return self._courseMagn

    def gpsSpeed(self):
        return self._gpsSpeed

    def apparentWindDirection(self):
        return self._apparentWindDir

    def apparentWindSpeed(self):
        return self._apparentWindSpeed

    # Simulator State

    def setActuators( self, sailAngle, rudderAngle ):
        self._sailAngle = sailAngle
        self._rudderAngle = rudderAngle
        self._physics.setActuators( sailAngle, rudderAngle )
    
    # Returns the boats sail angle
    def sail(self):
        return self._sailAngle

    # Returns the rudder angle
    def rudder(self):
        return self._rudderAngle

    def simulate(self, timestep, trueWind ):

        self._physics.simulate( timestep, trueWind )

        ( x, y ) = self._physics.utmCoordinate()

        apparentWind = self._physics.apparentWind()
        
        #(x,
         #apparentWind,
         #speed) = self.one_loop(timestep, trueWind )

        (self._latitude, self._longitude) = LLUTM.UTMtoLL(self._ref_ellipse, self._utmOriginY+y, self._utmOriginX+x, self._utmZone)

        #self._courseReal = wrapTo2Pi( -atan2( speed[1], speed[0] ) + np.pi / 2 ) * 180 / np.pi
        self._courseReal = wrapTo2Pi( -self._physics.heading() + np.pi / 2 ) * 180/ np.pi
        self._courseMagn = self._courseReal

        self._gpsSpeed = self._physics.speed()
        #self._gpsSpeed = hypot(speed[0], speed[1])
        self._apparentWindDir = wrapTo2Pi( -apparentWind.direction() + np.pi ) * 180 / np.pi
        self._apparentWindSpeed = apparentWind.speed()

        return apparentWind.direction()

    def one_loop(self, dt, trueWind):
        (df, apparentWind, speed) = f(np.array([self._x, self._y, self._heading, self._speed, self._rotationSpeed]), trueWind, self._sailAngle, self._rudderAngle)
        #self.x += dt*df
        self._x += df[0] * dt
        self._y += df[1] * dt
        self._heading += df[2] * dt
        self._speed += df[3] * dt
        self._rotationSpeed += df[4] * dt

        self._heading = wrapTo2Pi(self._heading)
        return (np.array([self._x, self._y, self._heading, self._speed, self._rotationSpeed]), apparentWind, speed)

    def get_graph_values(self):
        ( x, y ) = self._physics.utmCoordinate()
        heading = self._physics.heading()
        return (x, y, heading)