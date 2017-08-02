from network_interface import NetworkInterface

import socket
import sys
import select
from struct import *
from vessel import SailBoat

MESSAGE_TYPE_BOAT_DATA = 0
MESSAGE_TYPE_AIS_CONTACT = 1
MESSAGE_TYPE_TIS_CONTACT = 2

class ASRNetwork(NetworkInterface)
    def __init__( self, address, port):
        super(ASRNetwork, self).__init__(address, port)
        self._sailCommand = 0
        self._rudderCommand = 0
        self._latitude = 0.0
        self._longitude = 0.0
        self._heading = 0
        self._gpsCourse = 0
        self._gpsSpeed = 0
        self._windSpeed = 0
        self._windDirection = 0

    def setup(self):
        """
        Setups the network interface into a working state

        @returns boolean    - True if the network was setup correctly otherwise False is returned
        """

        # Setup the TCP socket
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Connect the socket to the port where the server is listening
        print('connecting to %s on port %s' % (serverAddr, serverPort))

        self._connected = True
        try:
            self._sock.connect( (self._serverAddress, self._serverAddress) )
            print("Connected to control system!")
            self._sock.setblocking(0)
        except socket.error as e:
            print('Socket error:', e)
            self._connected = False
        return self._connected

    def stop(self):
        """
        Stops the network interface in a controlled manner
        """
        if self._connected:
            self._sock.close()

    def readNetworkData(self):
        """
        Reads network data off the line
        """

        # Read actuator command
        readReady, writeReady, errors = select.select( [self._sock], [self._sock], [self._sock], 0.01 )

        receiveFormat = '=HHH'

        if len(readReady):
            data = self._sock.recv( 6 ) # 2 bytes for packet length, 2 bytes for rudder, and 2 bytes for sail
            if len(data) is 6:
                (length, self._rudderCommand, self._sailCommand) = unpack(receiveFormat, data)
        
        # Send data
        # The sending format is:
        #   Msg Type(B), Lat(f), Lon(f), Speed(f), Course(h), WindDir(h), WindSpeed(f), heading(h), rudder(h), sail(h)
        dataLength = 27
        sendFormat = '=HB3f2H1f3H'
        data = pack( sendFormat, int(dataLength), MESSAGE_TYPE_BOAT_DATA,
                     self._latitude, self._longitude, self._gpsSpeed, int(self._gpsCourse),
                     int(self._windDirection), self._windSpeed,
                     int(self._heading),
                     int(self._rudderCommand), int(self._sailCommand) )
        print("Sent boat data")
        self.sendData( data )
    
    def getSailCommand(self):
        """
        Returns the last received sail command

        @returns int
        """
        return self._sailCommand

    def getRudderCommand(self):
        """
        Returns the last received rudder command

        @returns int
        """
        return self._rudderCommand

    def getWaypoints(self):
        """
        Returns the last received list of waypoints, or empty if there are currently
        no waypoints. The list is ordered with the first waypoint at the front, and 
        last waypoint being the last in the list.

        @returns list of lat lon tuples(double, double)
        """
        return []

    def setCompassData(self, heading, pitch, roll):
        """
        Sets the compass data to send to the endpoint.
        """
        self._heading = heading

    def setGPSData(self, latitude, longitude, speed, course):
        """
        Sets the GPS data to send to the endpoint.
        """
        self._latitude = latitude
        self._longitude = longitude
        self._gpsSpeed = speed
        self._gpsCourse = course

    def setWindData(self, windDirection, windSpeed):
        """
        Sets the wind data to send to the endpoint.
        """
        self._windDirection = windDirection
        self._windSpeed = windSpeed

    def setAISData(self, marineTraffic):
        """
        Sends a marineTraffic contact
        """ 
        sendFormat = '=HBI3fH'

        dataLength = 19
        id = marineTraffic.id()
        (latitude, longtitude) = marineTraffic.position()
        course = marineTraffic.course()
        speed = marineTraffic.speed()
        print("Sent AIS data")
        data = pack( sendFormat, int(dataLength), MESSAGE_TYPE_AIS_CONTACT, 
                    int(id), latitude, longtitude, speed, int(course) )
        self.sendData( data )