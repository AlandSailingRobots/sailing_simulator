import json

from asr_network import ASRNetwork
from redpigeon_network import RedpigeonNetwork


class Config:
    def __init__( self, configPath ):
        self._configPath = configPath
        self._latitude = 0
        self._longitude = 0
        self._windDir = 0
        self._windSpd = 0
        self._boatUpdate_ms = 100
        self._aisUpdate_ms = 1000
        self._marineTraffic = []

    
    def load(self):
        """ 
        Loads the configuation

        @returns boolean    - True if the config was loaded
        """

        with open(configPath) as data_file:    
            config = json.load(data_file)

        if config.get("lat_origin") and config.get("lon_origin"):
            self._latitude = config["lat_origin"]
            self._longitude = config["lon_origin"]
        else:
            print("ERROR: Missing robot start position!")
            return False
        
        if config.get("wind_direction") and config.get("wind_speed"):
            self._windDir = config["wind_direction"]
            self._windSpd = config["wind_speed"]
        else:
            print("ERROR: Missing wind data!")
            return False
        
        if config.get("boat_update_ms"):
            self._boatUpdate_ms = config["boat_update_ms"];

        if config.get("ais_update_ms"):
            self._aisUpdate_ms = config["ais_update_ms"];

        return self.__setNetworkInterface(config)
        

    def robotLat(self):
        """
        Returns the sailing robot's starting latitude

        @returns double
        """
        return self._latitude

    def robotLon(self):
        """
        Returns the sailing robot's starting longitude

        @returns double
        """
        return self._longitude

    def windDirection(self):
        """
        Returns the true wind direction

        @returns int
        """
        return self._windDir

    def windSpeed(self):
        """
        Returns the true wind speed

        @returns int
        """
        return self._windSpd

    def robotUpdateTime(self):
        """
        Returns how often the sailing robot's sensor data should be sent out

        @returns int
        """
        return self._boatUpdate_ms

    def aisUpdateTime(self):
        """
        Returns how often the AIS data should be sent out

        @returns int
        """
        return self._aisUpdate_ms

    def networkInterface(self):
        """
        Returns the network interface that should be used for communication

        @returns NetworkInterface
        """
        return self._network

    def marineTraffic(self):
        """
        Returns the marine traffic that should be simulated

        @returns list of marine traffic
        """
        return self._marineTraffic

    def __setNetworkInterface(self, config):
        """
        Determines the network interface that should be used and sets it up
        """
        networkInterfaceStr = ""
        address = "localhost"
        port = 6900

        if config.get("network").get("interface"):
            networkInterfaceStr = config["network"]["interface"]

        if config.get("network").get("address"):
            address = config["network"]["address"]
        if config.get("network").get("port"):
            port = config["network"]["port"]

        if networkInterfaceStr == "RedPigeon":
            print("Using RedPigeon network interface")
            self._network = RedpigeonNetworkInterface(address, port)
        # We default to ASR network interface
        else if:
            print("Using ASR network interface")
            self._network = ASRNetworkInterface(address, port)

        return self._network.setup()       