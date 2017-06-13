
import math
from math import cos, sin, atan2, hypot

from geopy.distance import great_circle


RADIUS_OF_EARTH_KM = 6371.0


def distanceKM(lat, lon, lat1, lon1):
    latLon = (lat, lon)
    latLon1 = (lat1, lon1)
    return great_circle(latLon, latLon1).m


#def distanceKM(lat, lon, lat1, lon1):
#    print("Lat: " + str(lat) + " Lon: " + str(lon) + " Lat: " + str(lat1) + " Lon: " + str(lon1))
#    deltaLatRadians = math.radians(lat1 - lat)
#    deltaLonRadians = math.radians(lon1 - lon)
#    LatRadians = math.radians(lat)
#    Lat1Radians = math.radians(lat1)

#    a = sin(deltaLatRadians / 2)**2 + (cos(LatRadians) * cos(Lat1Radians)) + sin(deltaLonRadians / 2)**2
#    b = 2 * atan2( math.sqrt(a), math.sqrt(1 - a) )
#    print("Distance: " + str(RADIUS_OF_EARTH_KM * b))
#    return RADIUS_OF_EARTH_KM * b

    