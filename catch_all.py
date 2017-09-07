
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

		sigma = cos( phi_ap ) + cos( sail )
		if (sigma < 0):
			sail = np.pi - phi_ap
		else:
			sail = np.sign( sin(phi_ap) ) * abs( sail )
		return( rudder, sail, phi_ap, lat, lon )


def loadConfiguration(configPath, traffic):
    global AIS_UPDATE_MS
    global BOAT_UPDATE_MS

    with open(configPath) as data_file:
        config = json.load(data_file)
    boat_type   = config["boat_type"]
    boat_config = config["boat_config"]
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
        vessels.append(SailBoat( SailingPhysicsModel(0,0,0,boat_config),latOrigin,lonOrigin,0,0))
    else:
        vessels.append(SailBoat( ASPirePhysicsModel( 0,0,0,boat_config,trueWindDir + np.pi),latOrigin,lonOrigin,0,0))

    # Load Marine Traffic
    if traffic == 1:
        for marineVessel in config["traffic"]:
            id = marineVessel["mmsi"]
            lat = marineVessel["lat_origin"]
            lon = marineVessel["lon_origin"]
            heading = wrapTo2Pi(np.deg2rad(90 - marineVessel["heading"] )) # [-pi, pi] east north up
            speed = marineVessel["speed"]
            length = marineVessel["length"]
            beam = marineVessel["beam"]
            vessels.append(MarineTraffic(SimplePhysicsModel(heading, speed), lat, lon, heading, speed, id, length, beam))

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

    def getBearing( asv, vessel ):
        (asvLat, asvLon) = asv.position()
        (vesselLat, vesslLon) = asv.position()

        boatLatitudeInRadian = np.deg2rad(asvLat)
        waypointLatitudeInRadian = np.deg2rad(vesselLat)
        deltaLongitudeRadian = np.deg2rad(vesslLon - asvLon)

        y_coordinate = sin(deltaLongitudeRadian) * cos(waypointLatitudeInRadian)
        x_coordinate = cos(boatLatitudeInRadian) * sin(waypointLatitudeInRadian) - sin(boatLatitudeInRadian) * cos(waypointLatitudeInRadian) * cos(deltaLongitudeRadian)

        bearingToWaypointInRadian = atan2(y_coordinate, x_coordinate)
        bearingToWaypoint = np.rad2deg(bearingToWaypointInRadian)
        return wrapAngle(bearingToWaypoint)

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
        # if btw < 0:
        #     btw += 360
        # else:
        #     btw -= 360
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

    def boatInVisualRange( asv, vessel):
        bearing = getBearing(asv, vessel)

        bearingDiff = abs( getBearingDiff(asv.heading(), bearing) )

        if bearingDiff < CAMERA_ANGLE:
            return True
        return False

    def order_to_deg(command_rudder, command_sheet):
        if command_rudder > 8000 or command_rudder < 3000:
            command_sheet = 4215
            command_rudder = 5520
        return ((command_rudder-5520)*(np.pi/6.0)/1500.0,
                (command_sheet-4215)*(np.pi/-6.165)/900.0)