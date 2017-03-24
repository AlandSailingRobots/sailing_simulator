from vessel import Vessel
import LatLongUTMconversion as LLUTM

import numpy as np
from math import cos, sin, atan2, hypot

def wrapTo2Pi(theta):
    if theta < 0:
        theta += 2*np.pi
    theta = theta % (2*np.pi)
    return theta


class SailingBoat(Vessel):
    def __init__( self, lat, lon, course, speed ):
        self._latitude = lat
        self._longitude = lon
        self._course = course
        self._speed = speed
        self._x = 0
        self._y = 0
        self._rotationSpeed = 0
        self._accelerationDot = 0
        self._rotationSpeedDot = 0
        self._ref_ellipse = 23
        (self._utmZone, self._utmOrigin_x, self._utmOrigin_y) = LLUTM.LLtoUTM( self._ref_ellipse, self._latitude, self._longitude )

        self._sailAngle = 0
        self._rudderAngle = 0
        self._apparentWindAngle = 0
        self._apparentWindSpeed = 0
        self._driftCoefficient = 0.03
        self._tangentialFriction = 40    # kg s^-1
        self._angularFriction = 6000    # kg m
        self._sailLift = 500            # kg s^-1
        self._rudderLift = 2000         # kg s^-1
        self._distanceToSailCoE = 0.5   # m
        self._distanceToMast = 0.5      # m
        self._distanceToRudder = 2      # m
        self._mass = 300                # kg
        self._momentOfInertia = 400     # kg m^s
        self._rudderBreakCoefficient = 0.2

    def setSail( self, sailAngle ):
        self._sailAngle = self.getCorrectSailAngle (sailAngle )
    
    def setRudder( self, rudderAngle ):
        self._rudderAngle = rudderAngle

    def apparentWindAngle(self):
        return self._apparentWindAngle

    def apparentWindSpeed(self):
        return self.apparentWindSpeed

    def calculateApparentWind(self, truewindSpeed, trueWindAngle ):
        apparentWindVector = [trueWindSpeed * cos( trueWindAngle - course ) - speed, trueWindSpeed * sin(trueWindAngle - course)]

        self._apparentWindAngle = atan2(apparentWindVector[1], apparentWindVector[0])
        self._apparentWindSpeed = hypot(apparentWindVector[0], apparentWindVector[1])
        self._sailAngle = self.getCorrectSailAngle ( self._sailAngle )

    def get_graph_values(self):
        return (self._x, self._y, self._rotationSpeed)

    def simulatorTest(self, timestep, trueWindSpeed, trueWindAngle, rudder, sail):
        self.setSail(sail)
        self.setRudder(rudder)
        return self.simulate(timestep, trueWindSpeed, trueWindAngle )

    def simulate(self, timestep, trueWindSpeed, trueWindAngle ):
        #(state, dots) = calculateNextStateDelta( self._course, self._speed, self._rotationSpeed, trueWindSpeed, trueWindAngel, self._sailAngle, self._rudderAngle )
        (x_drift, y_drift) = self.calculateDrift( trueWindSpeed, trueWindAngle )

        forceOnSail = self.calculateForceOnSail()
        forceOnRudder = self.calculateForceOnRudder()

        acceleration_dot = self.calculateAcceleration( forceOnSail, forceOnRudder )
        rotation_dot = self.calculateRotationForce( forceOnSail, forceOnRudder )

        self._x += timestep * x_drift
        self._y += timestep * y_drift
        self._rotationSpeed += timestep* self._rotationSpeed
        self._accelerationDot += timestep * acceleration_dot
        self._rotationSpeedDot += timestep * rotation_dot

        self._rotationSpeed = wrapTo2Pi( self._rotationSpeed )

        return (np.array([self._x, self._y, self._rotationSpeed, self._accelerationDot, self._rotationSpeedDot]),
                self._apparentWindSpeed,
                self._apparentWindAngle,
                [x_drift, y_drift])

        #(self._latitude, self._longitude) = LLUTM.UTMtoLL( self._ref_ellipse, self._utmOrigin_x + self._x, self.utmOrigin_y + self._y, self.utmZone)



    # Calculate the drift of and change in position from the boat's current velocity and the force of
    # the wind on the sails
    def calculateDrift(self, trueWindSpeed, trueWindAngle ):
        x_boatVelocity = self._speed * cos(self._course)
        y_boatVelocity = self._speed * sin(self._course)

        x_windDrift = self._driftCoefficient * trueWindSpeed * cos(trueWindAngle)
        y_windDrift = self._driftCoefficient * trueWindSpeed * sin(trueWindAngle)

        x_drift = x_boatVelocity + x_windDrift
        y_drift = y_boatVelocity + y_windDrift

        return (x_drift, y_drift)

    # Check the sail is the right way around
    def getCorrectSailAngle (self, sailAngle):
        sigma = cos(self._apparentWindAngle) + cos(sailAngle)
        if (sigma < 0):
            return np.pi + self._apparentWindAngle
        elif sin(self._apparentWindAngle) is not 0:
            # Ensure the sail is on the right side of the boat
            return -np.sign( sin( self._apparentWindAngle ) ) * abs(sailAngle)
        else:
            return sailAngle

    def calculateForceOnSail(self):
        return self._sailLift * self._apparentWindSpeed * sin( self._sailAngle - self._apparentWindAngle )

    def calculateForceOnRudder(self):
        return self._rudderLift * self._speed * sin( self._rudderAngle )
    
    # The acceleration of the boat is affected by three forces, the wind on the sail, a braking force
    # from the water on the rudder, and a tangential friction force
    def calculateAcceleration (self, forceOnSail, forceOnRudder):

        sailAccelerationForce = forceOnSail * sin( self._sailAngle )
        rudderBrakeForce = self._rudderBreakCoefficient * forceOnRudder * sin( self._sailAngle )
        tangentialFictionForce = np.sign( self._speed ) * ( self._tangentialFriction * ( self._speed )**2 )

        return ( (sailAccelerationForce - rudderBrakeForce) - tangentialFictionForce) / self._mass

    # Rotation force is affected by the wind, rudder and the boats current rotational speed
    def calculateRotationForce(self, forceOnSail, forceOnRudder):
        sailRotationForce = forceOnSail * ( self._distanceToSailCoE - self._distanceToMast * cos( self._sailAngle ) )
        rudderRotationForce = self._distanceToRudder * forceOnRudder * cos( self._rudderAngle )
        rotationForce = self._angularFriction * self._rotationSpeed * abs( self._speed )

        return ( sailRotationForce - rudderRotationForce - rotationForce ) / self._momentOfInertia

