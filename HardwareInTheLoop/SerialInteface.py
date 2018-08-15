import serial

class SerialInterface:
    def __init__(self, portName="/dev/ttyUSB0"):
        self.portName = portName
        self.device = serial.Serial(portName)



    def writeWithId(self, devId, dataId,  data):
        toSendId = (devId << 4) + dataId
        toSendId = toSendId.to_bytes(1, "big")

        self.device.write(toSendId)
        self.sendIntger(data)
        #self.device.close()

    #
    def readData(self):
        return self.device.read(1)

    def sendIntger(self, data):
        mask = 0xff000000
        raw = data & mask
        raw = raw >> 24
        # print(raw.to_bytes(1, "big"))
        self.device.write(raw.to_bytes(1, "big"))

        for n in range(3):
            data = data << 8
            raw = data & mask
            raw = raw >> 24
            #print(hex(raw))
            self.device.write(raw.to_bytes(1, "big"))


