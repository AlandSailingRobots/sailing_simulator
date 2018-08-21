import serial


class Compass:
    def __init__(self, devId, serInterface):
        self.serInterface = serInterface
        self.devId = devId


    def writeHeading(self, heading):
        self.serInterface.writeWithId(self.devId, 1, int(10*heading))

