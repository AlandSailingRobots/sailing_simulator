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
MESSAGE_TYPE_TIS_FIELD     = 3
MESSAGE_TYPE_WINGBOAT_CMD  = 4
MESSAGE_TYPE_SAILBOAT_CMD  = 5
MESSAGE_TYPE_WAYPOINT_DATA = 6


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
            #self._sock.setblocking(0)
        except socket.error as e:
            print('Socket error:', e)
            self._connected = False

    def connected(self):
        return self._connected

    def readActuatorData(self, data):
        (rudderCmd, sailCmd) = (0,0)

        if len(data) is 9:
            (MESSAGE_TYPE, rudderCmd, sailCmd) = unpack('=B2f', data)
            
        return (rudderCmd, sailCmd)


    def receiveWaypoint(self, data):
        (longitude, latitude, declination, radius, prevLon, prevLat, prevDec, prevRad) = (0,0,0,0,0,0,0,0)
        receiveFormat = '=Bi2d4i2d2i'  # H=uint16_t, i=int, f=float, B = Byte

        if len(data) is 61:
            (MESSAGE_TYPE, nextId, longitude, latitude, declination, radius,
             staytime, prevId, prevLon, prevLat, prevDec, prevRad) = unpack(receiveFormat, data)

        return (longitude, latitude, declination, radius, prevLon, prevLat, prevDec, prevRad)
        

    # Packets up and sends the boat data across TCP
    def sendBoatData( self, sailboat,MESSAGE_TYPE ):
        # latitude, longitude, course, speed      # GPS
        (latitude, longitude) = sailboat.position()
        course = sailboat.course() # [-180, 180] degree east north up
        speed = sailboat.speed()
        windDir = sailboat.apparentWind().direction() # [-180, 180] degree, trigo, wind vector
        windSpeed = sailboat.apparentWind().speed()
        heading = sailboat.heading() # [-180, 180] degree east north up
        if MESSAGE_TYPE == MESSAGE_TYPE_SAILBOAT_DATA:
        # The sending format is:
            #   Msg Type(B), Lat(f), Lon(f), Speed(f), Course(h), WindDir(h), WindSpeed(f), heading(h), rudder(h), sail(h)
            sail, rudder = sailboat.physicsModel().getActuators()
            dataLength = 23
            sendFormat = '=HB3f2h1fh'
            data = pack( sendFormat, int(dataLength), MESSAGE_TYPE,
                 latitude, longitude, speed, int(course),
                 int(windDir), windSpeed,
                 int(heading) )

        elif MESSAGE_TYPE == MESSAGE_TYPE_WINGBOAT_DATA:
        # The sending format is:
            #   Msg Type(B), Lat(f), Lon(f), Speed(f), Course(h), WindDir(h), WindSpeed(f), heading(h), rudder(h), sail(h)
            tail, rudder = sailboat.physicsModel().getActuators()
            dataLength = 23
            tail = 0 # while the command is not implemented
            sendFormat = '=HB3f2h1fh'
            data = pack( sendFormat, int(dataLength), MESSAGE_TYPE,
                 latitude, longitude, speed, int(course),
                 int(windDir), windSpeed, 
                 int(heading) )   

        self.sendData( data )

    def sendAISContact( self, boat ):
        print ("sendAISContact")
        sendFormat = '=HBI3fh2f'
        #sendFormat = '=HBI3dh2f'
        #sendFormat = '=HBI2dfh2f'

        dataLength = 27
        id = boat.id()
        (latitude, longitude) = boat.position()
        print("Lat/lon before actual AIS data sending: ", latitude, " ", longitude)
        course = boat.course() # [-180, 180] east north up
        speed = boat.speed()
        length = boat.length()
        beam = boat.beam()
        data = pack( sendFormat, int(dataLength), MESSAGE_TYPE_AIS_CONTACT,
                     int(id), latitude, longitude, speed, int(course), length, beam )
        self.sendData( data )


#    def sendVisualContact( self, boat ):
#        sendFormat = '=HBI2f'

#        dataLength = 19
#        id = boat.id()
#        (latitude, longitude) = boat.position()
#        data = pack( sendFormat, int(dataLength), MESSAGE_TYPE_TIS_CONTACT,
#                     int(id), latitude, longitude )
#        self.sendData( data )

    def sendVisualField( self, relativeObstacleDistances, heading):
        sendFormat = '=HB24Hh'         #this depends on camera FOV = 24
        dataLength = calcsize(sendFormat)  
        data = bytearray(dataLength)
        offset = 0;
        # transmit the remaining length to read after the two bytes for the dataLength
        pack_into( '=HB', memoryview(data), offset, int(dataLength - 2), MESSAGE_TYPE_TIS_FIELD)
        offset += 3
        for i in range(24):
            pack_into('H', memoryview(data), offset, int(relativeObstacleDistances[i]))
            offset += 2
        print("Sending heading " + str(heading))
        pack_into('h', memoryview(data), offset, int(heading))
        self.sendData( data )

    def sendData( self, data ):
        readReady, writeReady, errors = select.select( [self._sock], [self._sock], [self._sock], 0.01 )
        if( len(writeReady) ):
            self._sock.sendall( data )


    def receiveData( self ):
        readReady, writeReady, errors = select.select([self._sock], [self._sock], [self._sock], 0.01)

        if len(readReady):
            (data_length,) = unpack('=H', self._sock.recv(2))
            data = self._sock.recv(data_length)
            return data
        else:
            return ""
