from threading import Thread
import  time
import sys
import math as m

sys.path.append('HardwareInTheLoop')

from SerialInteface import SerialInterface
from Compass import Compass
from Gps import Gps
from Windsensor import Windsensor
from Actuators import Actuators



class ViritualBoat (Thread):
    def __init__(self, lat, long, ip, serPort):
        Thread.__init__(self)
        self.serInterface = SerialInterface(serPort)
        self.gps = Gps(ip)
        self.actuators = Actuators(self.serInterface)
        self.compass = Compass(1, self.serInterface)
        self.windSensor = Windsensor(2, self.serInterface)

        self.latitude = lat
        self.longitude = long
        self.course = 0
        self.speed = 0
        self.heading = 0
        self.windSpeed = 0
        self.windDirection = 0

        self.rudderAngle = 0
        self.wingsailAngle = 0


    

    def run(self):
        while (True):
            self.sendNavigationData()
            time.sleep(0.6) #In order to grantie that the HW-nodes of the system gets all values


    def startActuators(self):
        self.actuators.start()

    def setNavigationParameters(self, sailboat):
        # latitude, longitude, course, speed      # GPS
        self.latitude = sailboat.position()[0]
        self.longitude = sailboat.position()[1]

        self.course = self.limitAngleRange(90-sailboat.course())# [-180, 180] degree east north up
        self.speed = sailboat.speed()
        self.windDirection = self.limitAngleRange(180-sailboat.apparentWind().direction()) # [-180, 180] degree, trigo, wind vector
        self.windSpeed = sailboat.apparentWind().speed()
        self.heading = self.limitAngleRange(90-sailboat.heading()) # [-180, 180] degree east north up

    def sendNavigationData(self):
        self.sending = True
        self.compass.writeHeading(self.heading)
        self.windSensor.writeWindDirection(self.windDirection)
        self.windSensor.writeWindSpeed(self.windSpeed)
        self.gps.writeGPS(self.latitude, self.longitude, self.speed, self.course)

    def getActuatorData(self):
        self.rudderAngle = self.actuators.getRudderAngle()
        self.wingsailAngle = self.actuators.getWingsailAngle()
        rudderRad =  -(2*m.pi*self.rudderAngle)/360
        wingRad = (2 * m.pi * self.wingsailAngle) / 360
        return (rudderRad, wingRad)


    def NEto360(self, val):
        if val < 0:
            val = val +360
        return val

    def limitAngleRange(self, val):
        return m.degrees(2 * m.atan(m.tan((m.radians(val) - m.pi) / 2)) + m.pi);