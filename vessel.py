import LatLongUTMconversion as LLUTM
import numpy as np

from physics_models import WindState, PhysicsModel, mainBoatPhysicsModel, ASPirePhysicsModel, SailingPhysicsModel
from utils import wrapTo2Pi, radTodeg

# MOVE TO UTIL FILE
class Vessel:
    def __init__(self, physicsModel, lat, lon, course, speed, length, beam):
        self._physicsModel = physicsModel
        self._course       = course
        # SET INITIAL MODEL COURSE
        self._elipseRef    = 23
        (self._utmZone, self._utmOriginX, self._utmOriginY) = LLUTM.LLtoUTM( self._elipseRef, lat, lon)
        self._speed = 0
        self._length = 0
        self._beam = 0

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

    def length(self):
        return self._length

    def beam(self):
        return self._beam


class MarineTraffic(Vessel):
    def __init__(self, physicsModel, lat, lon, course, speed, id, length, beam):
        self._physicsModel = physicsModel
        self._course       = course
        # SET INITIAL MODEL COURSE
        self._elipseRef    = 23
        (self._utmZone, self._utmOriginX, self._utmOriginY) = LLUTM.LLtoUTM( self._elipseRef, lat, lon)
        self._speed = 0
        self._id = id
        self._length = length
        self._beam = beam

    def course(self):
        return radTodeg( wrapTo2Pi( self._physicsModel.heading() ) ) # [-180, 180] east north up
    
    def speed(self):
        return self._physicsModel.speed()

    def id(self):
        return self._id

    def length(self):
        return self._length

    def beam(self):
        return self._beam


class SailBoat(Vessel):
    def __init__(self, physicsModel, lat, lon, course, speed):
        self._physicsModel = physicsModel
        self._course       = course
        self._elipseRef    = 23
        (self._utmZone, self._utmOriginX, self._utmOriginY) = LLUTM.LLtoUTM( self._elipseRef, lat, lon)

    def course(self):
        return self.heading()

    def heading(self):
        heading = radTodeg( wrapTo2Pi( self._physicsModel.heading() ) ) # [-180, 180] degree east north up
        return heading

    def speed(self):
        return self._physicsModel.speed()

    def apparentWind( self ):
        apparentWind = self._physicsModel.apparentWind()
        windDir      = radTodeg( wrapTo2Pi( apparentWind.direction() ) ) # [-180, 180] degree east north up
        windSpeed    = apparentWind.speed()
        return WindState( windDir, windSpeed )

    def sailAndRudder(self):
        """ if the boat chosen is ASPire, the sail == the tailWing """
        return self._physicsModel.getActuators()

    def getGraphValues( self ):
        (x, y)  = self._physicsModel.utmCoordinates()

        heading = self._physicsModel.heading()
        return ( x, y, heading )
