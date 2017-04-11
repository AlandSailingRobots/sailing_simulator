import numpy as np
from math import cos, sin, atan2, hypot


def wrapTo2Pi(theta):
    if theta < 0:
        theta += 2*np.pi
    theta = theta % (2*np.pi)
    return theta

class WindState(object):
    def __init__(self, windDirection, windSpeed):
        self._dir = windDirection
        self._spd = windSpeed

    def direction(self):
        return self._dir

    def speed(self):
        return self._spd

class PhysicsModel:
    # X and y are in UTM coordinates, the heading is in radians
    def __init__(self, x = 0, y = 0, heading = 0):
        self._x = x
        self._y = y
        self._heading = heading
        self._rotationSpeed = 0
        self._speed = 0

    def utmCoordinate(self):
        return (self._x, self._y)

    def heading(self):
        return self._heading

    def speed(self):
        return self._speed

    def simulate(self, timeDelta, trueWind):
        raise NotImplementedError("Must override simulate method!")


class SailingPhysicsModel(PhysicsModel):
    # X and y are in UTM coordinates, the heading is in radians
    def __init__(self, x = 0, y = 0, heading = 0):
        self._x = np.float64(x)
        self._y = np.float64(y)
        self._heading = np.float64(heading)
        self._rotationSpeed = np.float64(0)
        self._speed = 0
        self._sailAngle = 0
        self._rudderAngle = 0
        self._apparentWind = WindState(0, 0)
        self._driftCoefficient = 0.03
        self._tangentialFriction = 40     # kg s^-1
        self._angularFriction = 6000      # kg m
        self._sailLift = 500              # kg s^-1
        self._rudderLift = 2000           # kg s^-1
        self._distanceToSailCoE = 0.5     # m
        self._distanceToMast = 0.5        # m
        self._distanceToRudder = 2        # m
        self._boatMass = 300              # kg
        self._momentOfInertia = 400       # kg m^2
        self._rudderBreakCoefficient = 0.2   

    def setActuators(self, sail, rudder):
        self._sailAngle = sail
        self._rudderAngle = rudder

    def getActuators(self):
        return ( self._sailAngle, self._rudderAngle )

    def apparentWind(self):
        return self._apparentWind


    def simulate(self, timeDelta, trueWind):
        (x_dot, y_dot) = self.calculateDrift( trueWind )
        self.updateApparentWind( trueWind )
        self.calculateCorrectSailAngle()

        # The acceleration of the boat is affected by three forces, the wind on the sail, a braking force
        # from the water on the rudder, and a tangential friction force
        sailForce = self.forceOnSails()
        rudderForce = self.forceOnRudder()

        sailAccelerationForce = sailForce * sin( self._sailAngle )
        rudderBrakeForce = self._rudderBreakCoefficient * rudderForce * sin( self._sailAngle )
        tangentialFictionForce = np.sign(self._speed) * (self._tangentialFriction * (self._speed)**2)

        acceleration_dot = ( (sailAccelerationForce - rudderBrakeForce) - tangentialFictionForce) / self._boatMass

        sailRotationForce = sailForce * ( self._distanceToSailCoE - self._distanceToMast * cos( self._sailAngle ) )
        rudderRotationForce = self._distanceToRudder * rudderForce * cos( self._rudderAngle )
        rotationForce = self._angularFriction * self._rotationSpeed * abs( self._speed )

        rotationSpeed_dot = (sailRotationForce - rudderRotationForce - rotationForce) / self._momentOfInertia
        

        self._x += x_dot * timeDelta
        self._y += y_dot * timeDelta
        self._heading += self._rotationSpeed * timeDelta
        self._speed += acceleration_dot * timeDelta
        self._rotationSpeed += rotationSpeed_dot * timeDelta
        self._heading = wrapTo2Pi(self._heading)
    
    # Calculate the drift of and change in position from the boat's current velocity and the force of
    # the wind on the sails
    def calculateDrift(self, trueWind ):
        x_boatVelocity = self._speed * cos(self._heading)
        y_boatVelocity = self._speed * sin(self._heading)

        x_windDrift = self._driftCoefficient * trueWind.speed() * cos( trueWind.direction() )
        y_windDrift = self._driftCoefficient * trueWind.speed() * sin( trueWind.direction() )

        x_dot = x_boatVelocity + x_windDrift
        y_dot = y_boatVelocity + y_windDrift

        return (x_dot, y_dot)

    def updateApparentWind(self, trueWind ):
        apparentWindVector = [trueWind.speed() * cos( trueWind.direction() - self._heading ) - self._speed, trueWind.speed() * sin( trueWind.direction() - self._heading )]
        #print trueWind.direction() 
        #print trueWind.speed()
        #print apparentWindVector
        apparentWindAngle = atan2(apparentWindVector[1], apparentWindVector[0])
        #print apparentWindAngle
        apparentWindSpeed = hypot(apparentWindVector[0], apparentWindVector[1]) 
        #print apparentWindSpeed
        self._apparentWind = WindState( apparentWindAngle, apparentWindSpeed )

    # Ensures the sail is on the correct side of the boat
    def calculateCorrectSailAngle(self):
        sigma = cos( self._apparentWind.direction() ) + cos( self._sailAngle )
        if (sigma < 0):
            self._sailAngle = np.pi + self._apparentWind.direction()
        elif sin( self._apparentWind.direction() ) is not 0:
            # Ensure the sail is on the right side of the boat
            self._sailAngle = -np.sign( sin( self._apparentWind.direction() ) ) * abs( self._sailAngle )

    def forceOnSails(self):
        #print str(self._sailLift) + " " + str(self._apparentWind.speed()) + " : " + str( self._sailAngle ) + " " + str(self._apparentWind.direction() )
        return self._sailLift * self._apparentWind.speed() * sin( self._sailAngle - self._apparentWind.direction() )

    def forceOnRudder(self):
        return self._rudderLift * self._speed * sin( self._rudderAngle )

class SimplePhysicsModel(PhysicsModel):
     # X and y are in UTM coordinates, the heading is in radians
    def __init__(self, heading = 0, speed = 0, x = 0, y = 0):
        self._x = x
        self._y = y
        self._heading = heading
        self._rotationSpeed = 0
        self._speed = speed

    def simulate(self, timeDelta, trueWind):
        velocityX = self._speed * cos(self._heading)
        velocityY = self._speed * sin(self._heading)
        self._x += velocityX * timeDelta
        self._y += velocityY * timeDelta
        print("Position: " + str(self._x) + ", " + str(self._y) + " Speed: " + str(self._speed) + " Heading: " + str(self._heading));