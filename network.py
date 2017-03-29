import socket
import sys
import select
from struct import *

from physics_models import WindState, SailingPhysicsModel
from vessel import SailBoat


MESSAGE_TYPE_BOAT_DATA = 0


class BoatData:
    def __init__(self):
        self.latitude = 0.0
        self.longitude = 0.0
        self.heading = 0
        self.gpsCourse = 0
        self.speed = 0
        self.apparentWind = WindState( 0, 0 )

class Network:
    def __init__(self, serverAddr, serverPort):
        self._rudderCmd = 0
        self._sailCmd = 0

        # Setup the TCP socket
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Connect the socket to the port where the server is listening
        print('connecting to %s on port %s' % (serverAddr, serverPort))

        self._connected = True
        try:
            self._sock.connect( (serverAddr, serverPort) )
            print "Connected to control system!"
            self._sock.setblocking(0)
        except socket.error as e:
            print('Socket error:', e)
            self._connected = False

    def connected( self ):
        return self._connected

    def readActuatorData(self):
        readReady, writeReady, errors = select.select( [self._sock], [self._sock], [self._sock], 0.01 )

        receiveFormat = '=HHH'

        if len(readReady):
            data = self._sock.recv( 6 ) # 2 bytes for packet length, 2 bytes for rudder, and 2 bytes for sail
            if len(data) is 6:
                (length, self._rudderCmd, self._sailCmd) = unpack(receiveFormat, data)
        return (self._rudderCmd, self._sailCmd)

    # Packets up and sends the boat data across TCP
    def sendBoatData( self, sailboat ):
        # latitude, longitude, course, speed      # GPS
        #windDir, windSpeed, windTemp     # Wind
        #heading, pitch, roll,                   # Compass
        #sail, rudder                            # Arduino

        dataLength = 27
        (latitude, longitude) = sailboat.position()
        course = sailboat.course()
        speed = sailboat.speed()
        windDir = sailboat.apparentWind().direction()
        windSpeed = sailboat.apparentWind().speed()
        heading = sailboat.heading()
        sail = 0
        rudder = 0

        # The sending format is:
        #   Msg Type(B), Lat(f), Lon(f), Speed(f), Course(h), WindDir(h), WindSpeed(f), heading(h), rudder(h), sail(h)

        sendFormat = '=HB3f2H1f3H'
        data = pack( sendFormat, dataLength, MESSAGE_TYPE_BOAT_DATA,
                     latitude, longitude, speed, course,
                     windDir, windSpeed,
                     heading,
                     rudder, sail )
        self.sendData( data )

    def sendData( self, data ):
        readReady, writeReady, errors = select.select( [self._sock], [self._sock], [self._sock], 0.01 )

        if( len(writeReady) ):
            self._sock.sendall( data )
                                                                    


