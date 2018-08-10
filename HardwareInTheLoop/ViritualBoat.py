from threading import Thread

from SerialInteface import SerialInterface
from Compass import Compass
from Gps import Gps
from Windsensor import Windsensor
from Actuators import Actuators

class ViritualBoat (Thread):
    def __init__(self):
        Thread.__init__(self)
        self.serInterface = SerialInterface()
        self.gps = Gps("192.168.4.176")
        self.actuators = Actuators(self.serInterface)
        self.compass = Compass(1, self.serInterface)
        self.windSensor = Windsensor(2, self.serInterface)
        self.actuators.start()

    






