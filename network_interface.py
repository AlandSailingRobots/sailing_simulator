
class NetworkInterface
    def __init__( self, address, port):
        self._address = address
        self._port = port

    def setup(self):
        """
        Setups the network interface into a working state

        @returns boolean    - True if the network was setup correctly otherwise False is returned
        """
        raise NotImplementedError("Must override setup() method!")

    def stop(self):
        """
        Stops the network interface in a controlled manner
        """
        raise NotImplementedError("Must override stop() method!")
    
    def getSailCommand(self):
        """
        Returns the last received sail command

        @returns int
        """
        raise NotImplementedError("Must override readSailCommand() method!")

    def getRudderCommand(self):
        """
        Returns the last received rudder command

        @returns int
        """
        raise NotImplementedError("Must override readRudderCommand() method!")

    def getWaypoints(self):
        """
        Returns the last received list of waypoints, or empty if there are currently
        no waypoints. The list is ordered with the first waypoint at the front, and 
        last waypoint being the last in the list.

        @returns list of lat lon tuples(double, double)
        """
        raise NotImplementedError("Must override getWaypoints() method!")

    def setCompassData(self, heading, pitch, roll):
        """
        Sets the compass data to send to the endpoint.
        """
        raise NotImplementedError("Must override setCompassData(heading, pitch, roll) method!")

    def setGPSData(self, latitude, longitude, speed, course):
        """
        Sets the GPS data to send to the endpoint.
        """
        raise NotImplementedError("Must override setGPSData(latitude, longitude, speed, course) method!")

    def setWindData(self, windDirection, windSpeed):
        """
        Sets the wind data to send to the endpoint.
        """
        raise NotImplementedError("Must override setWindData(windDirection, windSpeed) method!")

    def setAISData(self, marineTrafficList):
        """
        Sets the list of MarineTraffic objects to send to the endpoint
        """ 
        raise NotImplementedError("Must override setAISData(marineTrafficList) method!")  