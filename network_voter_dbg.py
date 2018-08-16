import socket
import sys
import select
from struct import *
import numpy as np


class VoterData:
    def __init__(self):
        self.courses = []
        self.veto = []


class VoterNetwork:
    def __init__(self, serverAddr, serverPort):
        self._courses = []
        self._veto = []

        self._name = ""
        self._weight = 0

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

    def receiveVoterData(self, data):
        receiveFormat = '=360h360?1f20c'  # h=int16_t, ?=bool, f=float, c=char
        #courses = np.zeros(360,np.int16)
        #veto = np.zeros(360,np.bool)
        #unpacked_data = []
        courses = []
        veto = []
        weight = 0

        if (len(data) == 1104):
            print("Unpacking")
            unpacked_data = unpack(receiveFormat, data)
        else:
            print("Voter debug data packet size not matching the expected size")
            print("Size of data is: ", len(data))

        for i in range(360):
            courses.append(unpacked_data[i])
            veto.append(unpacked_data[360+i])
            
        weight = unpacked_data[-21]
        name=unpacked_data[-20:]
        

        return (courses,veto,weight,name)


    def sendTrash( self ):
        sendFormat = '=HBB'

        dataLength = 2
        trash = 22
        data = pack( sendFormat, int(dataLength), trash, trash)
        self.sendData( data )


    def sendData( self, data ):
        readReady, writeReady, errors = select.select( [self._sock], [self._sock], [self._sock], 0.01 )
        if( len(writeReady) ):
            self._sock.sendall( data )


    def receiveData( self ):
        readReady, writeReady, errors = select.select([self._sock], [self._sock], [self._sock], 0.01)
        #readReady=[1]
        if len(readReady):
            (data_length,) = unpack('=H', self._sock.recv(2))
            data = self._sock.recv(data_length)
            return data
        else:
            return ""
    