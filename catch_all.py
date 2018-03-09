
# Sorry temporary solution

from physics_models import SimplePhysicsModel,SailingPhysicsModel, WindState, ASPirePhysicsModel
from vessel import Vessel,SailBoat, MarineTraffic
import json
from utils import *

def get_graph_values( sailBoat,boat_type ):
	(sail, rudder) = sailBoat.sailAndRudder() # if boat_type == 1 : sail == tailWing
	phi_ap = sailBoat.apparentWind().direction()
	(lat, lon) = sailBoat.position()
	if boat_type == 1:
		MWAngle   = sailBoat.physicsModel().MWAngle()
		tailAngle = sail
		return( rudder, tailAngle, phi_ap, lat, lon, MWAngle )
	else:
		return( rudder, sail, phi_ap, lat, lon )


def loadConfiguration(configPath, traffic):
    global AIS_UPDATE_MS
    global BOAT_UPDATE_MS

    # with open(configPath) as data_file:
    #     config = json.load(data_file)
    config = loadConfigFile(configPath)
    
    boat_config_path = config["boat_config"]
    boat_config = loadConfigFile(boat_config_path)
    boat_type   = boat_config["boat_type"]

    latOrigin = config["lat_origin"]
    lonOrigin = config["lon_origin"]
    sim_step  = config["simulation_step"]

    if config.get("boat_update_ms"):
        BOAT_UPDATE_MS = config["boat_update_ms"]

    if config.get("ais_update_ms"):
        AIS_UPDATE_MS = config["ais_update_ms"]

    print("Boat Update ms: " + str(BOAT_UPDATE_MS) + " AIS Update ms: " + str(AIS_UPDATE_MS))

    vessels = []

    trueWindDir = wrapTo2Pi(np.deg2rad(90 - config["wind_direction"])) # [-pi, pi] east north up
    print ("True Wind:" + str(trueWindDir))
    trueWindSpeed = config["wind_speed"]
    print(latOrigin, lonOrigin)
    # vessels.append(SailBoat( SailingPhysicsModel(), latOrigin, lonOrigin, 0, 0 ))
    if boat_type == 0:
        vessels.append(SailBoat( SailingPhysicsModel(0,0,0,boat_config_path),latOrigin,lonOrigin,0,0))
    else:
        vessels.append(SailBoat( ASPirePhysicsModel( 0,0,0,boat_config_path,trueWindDir + np.pi),latOrigin,lonOrigin,0,0))

    # Load Marine Traffic
    if traffic == 1:
        for marineVessel in config["traffic"]:
            if marineVessel["mmsi"] >= 100000000:
                id = marineVessel["mmsi"]
                lat = marineVessel["lat_origin"]
                lon = marineVessel["lon_origin"]
                heading = wrapTo2Pi(np.deg2rad(90 - marineVessel["heading"] )) # [-pi, pi] east north up
                speed = marineVessel["speed"]
                length = marineVessel["length"]
                beam = marineVessel["beam"]
                vessels.append(MarineTraffic(SimplePhysicsModel(heading, speed), lat, lon, heading, speed, id, length, beam))
            elif marineVessel["mmsi"] < 100000000:
                id = marineVessel["mmsi"]
                lat = marineVessel["lat_origin"]
                lon = marineVessel["lon_origin"]
                heading = marineVessel.get("heading", 0)
                heading = wrapTo2Pi(np.deg2rad(90 - heading )) # [-pi, pi] east north up
                speed = marineVessel.get("speed", 0)
                vessels.append(MarineTraffic(SimplePhysicsModel(heading, speed), lat, lon, heading, speed, id, 0, 0))

    return ( boat_type, sim_step,vessels, WindState( trueWindDir, trueWindSpeed ) )


class Functions:


    def get_to_socket_value( sailBoat ):
        heading = sailBoat.heading()
        (lat, lon) = sailBoat.position()
        course = sailBoat.course()
        speed = sailBoat.speed()

        gps = (lat, lon, course, heading, speed)

        windsensor = ( sailBoat.apparentWind().speed(), sailBoat.apparentWind().direction() )
        return (heading, gps, windsensor)

    def get_graph_values( sailBoat ):
        (sail, rudder) = sailBoat.sailAndRudder()
        phi_ap = sailBoat.apparentWind().direction()

        sigma = cos( phi_ap ) + cos( sail )
        if (sigma < 0):
            sail = np.pi + phi_ap
        else:
            if sin(phi_ap)is not 0:
                sail = -np.sign( sin(phi_ap) ) * abs( sail )
        (lat, lon) = sailBoat.position()
        return ( rudder, sail, phi_ap, lat, lon )

    def getDTW(asvpos,wppos):
        radiusEarth = 6371
        (asvLon, asvLat) = asvpos
        (wpLon, wpLat) = wppos
        deltaLat = np.deg2rad(wpLat-asvLat)
        asvLat = np.deg2rad(asvLat)
        wpLat = np.deg2rad(wpLat)
        deltaLon = np.deg2rad(wpLon-asvLon)

        tmp = np.sin(deltaLat/2)*np.sin(deltaLat/2) + \
            np.cos(asvLat)*np.cos(wpLat) * \
            np.sin(deltaLon/2)*np.sin(deltaLon/2)
        tmp = 2 * np.arctan2(np.sqrt(tmp), np.sqrt(1-tmp))
        dtw = radiusEarth * tmp*1000
        return dtw

    def getBTW(asvpos, wppos):
        (asvLon, asvLat) = asvpos
        (vesselLon, vesselLat) = wppos
        boatLatitudeInRadian = np.deg2rad(asvLat)
        waypointLatitudeInRadian = np.deg2rad(vesselLat)
        deltaLongitudeRadian = np.deg2rad(vesselLon - asvLon)

        y_coordinate = sin(deltaLongitudeRadian) * cos(waypointLatitudeInRadian)
        x_coordinate = cos(boatLatitudeInRadian) * sin(waypointLatitudeInRadian) - sin(boatLatitudeInRadian) * cos(waypointLatitudeInRadian) * cos(deltaLongitudeRadian)

        bearingToWaypointInRadian = atan2(y_coordinate, x_coordinate)
        btw = np.rad2deg(bearingToWaypointInRadian)
        return wrapAngle(btw)

    def getBearingDiff( h1, h2 ):
        diff = h2 - h1
        absDiff = abs( diff )

        if (absDiff <= 180):
            if absDiff == 180:
                return absdiff
            else:
                return diff

        elif (h2 > h1):
            return absDiff - 360
        else:
            return 360 - absDiff

    def boatInVisualRange(asv, vessel, cameraFOV):
        bearing = Functions.getBTW(asv.position(), vessel.position())
        print("boat in visual range")
        print("bearing: " + str(bearing))

        bearingDiff = abs( Functions.getBearingDiff(asv.heading(), bearing) )
        print("bearingDiff: " + str(bearingDiff))

        if bearingDiff < (cameraFOV/2):
            return True
        return False


    def replaceRelDistanceIfSmaller(relativeObstacleDistances, relativeDist, index):
        if index < 0:
            return
        if index >= len(relativeObstacleDistances):
            return
        if relativeDist < relativeObstacleDistances[index]:
            relativeObstacleDistances[index] = relativeDist


    def relativeDistancesFromBearingDistances(visualBearingsDistances, maxVisibleDistance, cameraFOV):
        relativeObstacleDistances = []
        for i in range(cameraFOV):
            relativeObstacleDistances.append(int(100))
        for [bearing, distance] in visualBearingsDistances:
            relDistance = 100
            degreeRange = 1
            if distance < maxVisibleDistance:
                relDistance = 100 * distance/ maxVisibleDistance;
                degreeRange = 10 * (maxVisibleDistance - distance)/ maxVisibleDistance;
            for i in range(int(degreeRange)):
                Functions.replaceRelDistanceIfSmaller(relativeObstacleDistances, relDistance, int(bearing + cameraFOV/2 + i))
                Functions.replaceRelDistanceIfSmaller(relativeObstacleDistances, relDistance, int(bearing + cameraFOV/2 - i))
        return relativeObstacleDistances
 




