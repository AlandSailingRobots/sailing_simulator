from threading import Thread
import time

class Actuators(Thread):
    def __init__(self, serInterface):
        Thread.__init__(self)
        self.serInterface = serInterface
        self.buff = b''
        self.rudderAngle = 0
        self.wingsailAngle = 0
        self.startSeq = [0xfb, 0xfa, 0xcb]

    def run(self):
        while (True):
            self.buff = b''
            message = b''
            while (message != b'\n'):

                message = self.serInterface.readData()
                self.buff += message
                if (len(self.buff) > 2):
                    if (self.isStart(self.buff[-3:])):
                        self.buff = self.buff[:-3]
                        self.readActuatorValues()


            print(self.buff[:-1])

    def isStart(self, seq):
        for i in range(len(seq)):
            if (seq[i] != self.startSeq[i]):
                return False
        #print ("IDENTFEIED START")
        return True

    def readActuatorValues(self):
        self.rudderAngle = self.readFloat()
        self.wingsailAngle = self.readFloat()

    def readFloat(self):
        floatStr = b''
        raw = self.serInterface.readData()
        while(raw!=b'\x00'):
            floatStr = floatStr + raw
            raw = self.serInterface.readData()
        return float(floatStr)

    def getRudderAngle(self):
        return self.rudderAngle

    def getWingsailAngle(self):
        return self.wingsailAngle