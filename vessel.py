import LatLongUTMconversion as LLUTM
import numpy as np
from physics_models import WindState, PhysicsModel

# MOVE TO UTIL FILE
def wrapTo2Pi(theta):
    if theta < 0:
        theta += 2*np.pi
    theta = theta % (2*np.pi)
    return theta

class Vessel:
    def __init__(self, physicsModel, lat, lon, course, speed):
        self._physicsModel = physicsModel
        self._course = course
        # SET INITIAL MODEL COURSE
        self._elipseRef = 23
        (self._utmZone, self._utmOriginX, self._utmOriginY) = LLUTM.LLtoUTM( self._elipseRef, lat, lon)
        self._speed = 0

    def physicsModel(self):
        return self._physicsModel

    # Returns the latitude and longitude position of the vessel
    def position(self):
        (x, y) = self._physicsModel.utmCoordinate()
        return LLUTM.UTMtoLL(self._elipseRef, self._utmOriginY+y, self._utmOriginX+x, self._utmZone)

    def course(self):
        return self._course

    def speed(self):
        return self._speed

class SailBoat(Vessel):
    def __init__(self, physicsModel, lat, lon, course, speed):
        self._physicsModel = physicsModel
        self._course = course
        self._elipseRef = 23
        (self._utmZone, self._utmOriginX, self._utmOriginY) = LLUTM.LLtoUTM( self._elipseRef, lat, lon)

        # Currently just returns the boat's compass heading, this will include current/tidal drift 
        # eventually TODO
    def course(self):
        return self.heading()

    def heading(self):
        heading = wrapTo2Pi( -self._physicsModel.heading() + np.pi / 2 ) * 180 / np.pi
        return heading

    def speed(self):
        return self._physicsModel.speed()

    def apparentWind( self ):
        apparentWind = self._physicsModel.apparentWind()
        windDir = wrapTo2Pi( -apparentWind.direction() + np.pi ) * 180 / np.pi
        windSpeed = apparentWind.speed()
        return WindState( windDir, windSpeed )

    def sailAndRudder(self):
        return self._physicsModel.getActuators()

    def getGraphValues( self ):
        (x, y) = self._physicsModel.utmCoordinates()
        heading = self._physicsModel.heading()
        return ( x, y, heading )

