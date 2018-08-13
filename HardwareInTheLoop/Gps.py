import socket
import time



class Gps():


    def __init__(self, ip):

        self.UDP_IP = ip
        self.UDP_PORT = 49194

        self.lat = 60.10855
        self.long = 19.92261
        self.speed = 0.0
        self.track = 0.0

        self.msgGGA = "72:$GPGGA,145135,6006.1683,N,01955.7536,E,2,07,1.20,58.15,M,21.223,M,,*78"
        self.msgRMC = self.crateRMC(self.lat, self.long, self.speed, self.track)
        self.msgGSA = "53:$GPGSA,A,3,8,32,18,14,11,10,120,,,,,,8.9,1.2,7.3*33"
        self.msgZDA = "38:$GPZDA,145136.00,23,07,2018,00,00*6F"
        self.msgGSV = "70:$GPGSV,2,1,07,08,62,222,45,32,31,132,48,18,49,268,28,28,23,322,16*70"
        self.socket = socket.socket(socket.AF_INET,  socket.SOCK_DGRAM) # UDP
        self.defultMsgs = [self.msgGGA, self.msgGSA, self.msgGSV, self.msgZDA]

    def sendData(self):
        for msg in self.defultMsgs:
            MESSAGE = bytes(msg, "ascii") + b'\x0d' + b'\x0a'
            self.socket.sendto(MESSAGE, (self.UDP_IP, self.UDP_PORT))
        MESSAGE = bytes(self.msgRMC, "ascii") + b'\x0d' + b'\x0a'
        self.socket.sendto(MESSAGE, (self.UDP_IP, self.UDP_PORT))



    def crateRMC(self, lat, long, speed, track):
        timeS =time.strftime("%H%M%S",time.gmtime())
        latS = self.degDec2degMinDec(lat)
        longS = self.degDec2degMinDec(long)
        speedS = str(speed)
        trackS = str(track)
        dateS= time.strftime("%d%m%y",time.gmtime())

        data = "GPRMC,"+ timeS +",A,"+latS+ ",N,"+longS+",E,"+speedS+","+trackS+","+dateS+",,"
        CS = self.calcChecksum(data)

        RMC = str(len(data)+2)+":$"+data + '*'+CS
        return RMC

    def setRMC(self, lat=-1, long=-1, speed=-1, track=-1):

        if lat != -1:
            self.lat= lat
        if long != -1:
            self.long= long
        if speed != -1:
            self.speed = speed
        if track != -1:
            self.track = track

        self.msgRMC = self.crateRMC(self.lat, self.long, self.speed, self.track)

    def degDec2degMinDec(self, cord):
        deg = str(int (cord - cord%1))

        min = round((cord*60)%60, 4)
        minInt = str(int(min - min%1))
        if len(minInt) < 2:
            minInt = "0" + minInt
        minDec = str(min%1)[1:]
        return deg+minInt+minDec

    def calcChecksum(self, data):
        CS = 0
        for n in data:
            CS = CS ^ ord(n)
        return hex(CS).upper()[2:]


    def writeGPS(self, lat, long, speed, track):
        self.setRMC(lat, long, speed, track)
        self.sendData()

