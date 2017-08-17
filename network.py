import socket
import sys
import select
from struct import *
from utils import wrapAngle
from physics_models import WindState, SailingPhysicsModel
from vessel import SailBoat


MESSAGE_TYPE_SAILBOAT_DATA = 0
MESSAGE_TYPE_WINGBOAT_DATA = 1
MESSAGE_TYPE_AIS_CONTACT   = 2
MESSAGE_TYPE_TIS_CONTACT   = 3



class BoatData:
    def __init__(self):
        self.latitude = 0.0
        self.longitude = 0.0
        self.heading = 0
        self.gpsCourse = 0
        self.speed = 0
        self.apparentWind = WindState(0, 0)


class Network:
    def __init__(self, serverAddr, serverPort):
        self._rudderCmd = 0
        self._sailCmd = 0

        self._nextId = 0
        self._longitude = 0
        self._latitude = 0
        self._declination = 0
        self._radius = 0
        self._staytime = 0
        self._prevId = 0
        self._prevLon = 0
        self._prevLat = 0
        self._prevDec = 0
        self._prevRad = 0
        # Setup the TCP socket
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Connect the socket to the port where the server is listening
        print('connecting to %s on port %s' % (serverAddr, serverPort))

        self._connected = True
        try:
            self._sock.connect((serverAddr, serverPort))
            print("Connected to control system!")
            self._sock.setblocking(0)
        except socket.error as e:
            print('Socket error:', e)
            self._connected = False

    def connected(self):
        return self._connected

    def readActuatorData(self):
        readReady, writeReady, errors = select.select([self._sock], [self._sock], [self._sock], 0.01)

        receiveFormat = '=H2d'

        if len(readReady):
            data = self._sock.recv(18)  # 2 bytes for packet length, 2 bytes for rudder, and 2 bytes for sail
            if len(data) is 18:
                (length, self._rudderCmd, self._sailCmd) = unpack(receiveFormat, data)
            print("length: ",length, " rudderCommand: ",self._rudderCmd, " tailCommand: ", self._sailCmd) 
        return (self._rudderCmd, self._sailCmd)

    def receiveWaypoint(self):
        readReady, writeReady, errors = select.select( [self._sock], [self._sock], [self._sock], 0.01 )
        receiveFormat = '=Hi2d4i2d2i'  # H=uint16_t, i=int, f=float, B = Byte

        if len(readReady):
            data = self._sock.recv(62)
            if len(data) is 62:
                # print("HEJ")
                (length, self._nextId, self._longitude, self._latitude, self._declination, self._radius,
                 self._staytime, self._prevId, self._prevLon, self._prevLat, self._prevDec, self._prevRad) = unpack(receiveFormat, data)
        return (self._longitude, self._latitude, self._declination, self._radius,
                self._prevLon, self._prevLat, self._prevDec, self._prevRad)

    # Packets up and sends the boat data across TCP
    def sendBoatData( self, sailboat,MESSAGE_TYPE ):
        # latitude, longitude, course, speed      # GPS
        (latitude, longitude) = sailboat.position()
        course = wrapAngle(sailboat.course())
        speed = sailboat.speed()
        windDir = wrapAngle(sailboat.apparentWind().direction())
        windSpeed = sailboat.apparentWind().speed()
        heading = wrapAngle(sailboat.heading())
        if MESSAGE_TYPE == MESSAGE_TYPE_SAILBOAT_DATA:
        # The sending format is:
            #   Msg Type(B), Lat(f), Lon(f), Speed(f), Course(h), WindDir(h), WindSpeed(f), heading(h), rudder(h), sail(h)
            sail, rudder = sailboat.physicsModel().getActuators()
            dataLength = 27
            sendFormat = '=HB3f2H1f3H'
            
            #print('latitude:',latitude,'longitude:',longitude,'speed:',speed, 'course:',course, 'windDir:',windDir,'windSpeed:',windSpeed,'heading:',heading,'rudder:',rudder,'sail:', sail)
            data = pack( sendFormat, int(dataLength), MESSAGE_TYPE,
                         latitude, longitude, speed, int(course),
                         int(windDir), windSpeed,
                         int(heading),
                         int(rudder), int(sail) )

        elif MESSAGE_TYPE == MESSAGE_TYPE_WINGBOAT_DATA:
        # The sending format is:
            #   Msg Type(B), Lat(f), Lon(f), Speed(f), Course(h), WindDir(h), WindSpeed(f), heading(h), rudder(h), sail(h)
            tail, rudder = sailboat.physicsModel().getActuators()
            dataLength = 27
            tail = 0 # while the command is not implmented
            sendFormat = '=HB3f2H1f3H'

            #print('latitude:',latitude,'longitude:',longitude,'speed:',speed, 'course:',course, 'windDir:',windDir,'windSpeed:',windSpeed,'heading:',heading,'rudder:',rudder,'tail:', tail)
            data = pack( sendFormat, int(dataLength), MESSAGE_TYPE,
                         latitude, longitude, speed, int(course),
                         int(windDir), windSpeed,
                         int(heading),
                         int(rudder), int(tail) )   
        print("Sent boat data")
        self.sendData( data )

    def sendAISContact( self, boat ):
        sendFormat = '=HBI3fH2f'

        dataLength = 27
        id = boat.id()
        (latitude, longitude) = boat.position()
        course = wrapAngle(boat.course())
        speed = boat.speed()
        print("Sent AIS data")
        length = boat.length()
        beam = boat.beam()
        # print("Sent AIS data")
        data = pack( sendFormat, int(dataLength), MESSAGE_TYPE_AIS_CONTACT,
                     int(id), latitude, longitude, speed, int(course), length, beam )
        self.sendData( data )

    def sendVisualContact( self, boat ):
        sendFormat = '=HBI2f'

        dataLength = 19
        id = boat.id()
        (latitude, longtitude) = boat.position()
        # print("Sent Visual data")
        data = pack( sendFormat, int(dataLength), MESSAGE_TYPE_TIS_CONTACT,
                     int(id), latitude, longitude )
        self.sendData( data )

    def sendData( self, data ):
        readReady, writeReady, errors = select.select( [self._sock], [self._sock], [self._sock], 0.01 )

        if( len(writeReady) ):
            self._sock.sendall( data )
