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
    def __init__(self, lat, long):
        Thread.__init__(self)
        self.serInterface = SerialInterface()
        self.gps = Gps("192.168.4.176")
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
        self.actuators.start()
        while (True):
                self.compass.writeHeading(self.heading)
                self.windSensor.writeWindDirection(self.windDirection)
                self.windSensor.writeWindSpeed(self.windSpeed)
                self.gps.writeGPS(self.latitude, self.longitude, self.speed, self.course)

                self.rudderAngle = self.actuators.getRudderAngle()
                self.wingsailAngle = self.actuators.getWingsailAngle()


                time.sleep(0.25)




    def setNavigationParameters(self, sailboat):
        # latitude, longitude, course, speed      # GPS
        self.latitude = sailboat.position()[0]
        self.longitude = sailboat.position()[1]
        #print(sailboat.course())
        self.course = self.limitAngleRange(90-sailboat.course())# [-180, 180] degree east north up
        #print(self.course)
        self.speed = sailboat.speed()
        self.windDirection = self.limitAngleRange(180-sailboat.apparentWind().direction()) # [-180, 180] degree, trigo, wind vector
        self.windSpeed = sailboat.apparentWind().speed()
        #print("SIMU WIND " + str(sailboat.apparentWind().direction()))
        self.heading = self.limitAngleRange(90-sailboat.heading()) # [-180, 180] degree east north up
        #print ("WIND " + str(self.windDirection))


    def getActuatorData(self):
        rudderRad =  (2*m.pi*self.rudderAngle)/360
        wingRad = (2 * m.pi * self.wingsailAngle) / 360
        return (rudderRad, wingRad)


    def NEto360(self, val):
        if val < 0:
            val = val +360
        return val

    def limitAngleRange(self, val):
        return m.degrees(2 * m.atan(m.tan((m.radians(val) - m.pi) / 2)) + m.pi);