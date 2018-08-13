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

        self.latitude = 0
        self.longitude = 0
        self.course = 0
        self.speed = 0
        self.heading = 0
        self.windSpeed = 0
        self.windDirection = 0



    

    def run(self):
        self.actuators.start()
        while (True):
                compass.writeHeading(95)
                windsensor.writeWindDirection(50)
                windsensor.writeWindSpeed(10)

                gps.setRMC(lat=lat, long=long)

                print("RUDDER " + str(actuatros.getRudderAngle()))
                print("WINGSAIL " + str(actuatros.getWingsailAngle()))
                time.sleep(0.25)



