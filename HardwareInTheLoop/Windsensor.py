

class Windsensor:
    def __init__(self, devId, serInterface):
        self.devId = devId
        self.serInterface = serInterface
        self.speedId = 1
        self.directionId = 2


    def writeWindSpeed(self, speed):
        self.serInterface.writeWithId(self.devId, self.speedId, speed)

    def writeWindDirection(self, dir):
        self.serInterface.writeWithId(self.devId, self.directionId, dir)
