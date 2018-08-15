import math as m

class Windsensor:
    def __init__(self, devId, serInterface):
        self.devId = devId
        self.serInterface = serInterface
        self.speedId = 1
        self.directionId = 2


    def writeWindSpeed(self, speed):
        self.serInterface.writeWithId(self.devId, self.speedId, int(speed*100))

    def writeWindDirection(self, dir):
        rad = ((dir *2*m.pi)/360)*10000
        rad = int(rad)
        self.serInterface.writeWithId(self.devId, self.directionId, rad)
