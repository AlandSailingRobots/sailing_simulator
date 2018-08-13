import sys

sys.path.append('HardwareInTheLoop')


from SerialInteface import SerialInterface
from Compass import Compass
from Gps import Gps
from Windsensor import Windsensor
from Actuators import Actuators

import time
import math as m

midLat = 60.10941
midLong = 19.92198
radius = 0.001

serInterface = SerialInterface()
compass = Compass(1,serInterface)
windsensor = Windsensor(2, serInterface)
actuatros = Actuators(serInterface)
#actuatros.start()

gps = Gps("192.168.4.176")

