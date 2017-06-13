SAILING_SIMULATOR
=================

This repository Aland sailing robot's simulator. The simulator will simulate a sailing ASV and provides a basic simulation of other marine traffic. The GPS tracks of the ASV and the simulated track are outputted in .track files, which the first one being the ASV.

## Usage

    ./simulation_main.py [config]

    Arguements:

        * [config] - Optional:  This is the path to the config file, if no path is included, the simulator will default to ./config.json

## Required Packages:

* numpy
* matplotlib
* geopy
* python-tk

### Arch Linux

On Arch linux as sudo:

    $ pacman -Syy && pacman -S python-pip
    $ pip install numpy
    $ pip install matplotlib
    $ pip install geopy
    $ pacman -S tk 

In some cases the install for *matplotlib* is corrupted in that case you need to build it:

    $ pip uninstall matplotlib
    $ git clone https://github.com/matplotlib/matplotlib.git
    $ cd matplotlib
    $ python setup.py install

### Ubuntu

On Ubuntu:

    $ apt-get install python-pip
    $ pip install numpy
    $ pip install matplotlib
    $ pip install geopy
    $ apt-get python-tk

## Using the simulator with the Aland code base

The Aland control system's simulator node creates a TCP server which the simulator connects to, this means the control system (Built in simulation mode) needs to be started first. Then the simulator can be started, it will automatically connect to the control system, at which point the simulation will begin.

## Simulator Configuration

The configuration file controls the starting position of the ASV, as well as how often the simulator will send update messages and simulated traffic.

### ASV State

The ASV's initial state is controlled by 4 configurable variables

    * "lat_origin" : A double, controls the starting latitude
    * "lon_origin": A double, controls the starting longitude
    * "wind_direction": A integer in degrees, controls the wind direction
    * "wind_speed": A integer in metres, controls the winds speed

### Message Updates

    * "boat_update_ms": A integer in milliseconds, controls how often boat state messages are sent out
    * "ais_update_ms": A integer in milliseconds, controls how often AIS contact messages are sent out

### Simulated Marine traffic

Simulated marine traffic can be defined in the configuration message, with the json array type with the key "traffic". Each entry describes a single simulated marine vehicle. 

A single marine vehicle is defined as:

        * "mmsi": The id of the vessel, an integer, used to uniquely id simulated traffic. An id below 100000000 will be regarded as a thermal imaging contact and not a AIS contact.
        * "heading": An integer, in degress, the heading of the vessel
        * "lat_origin": A double, controls the starting latitude of the marine vehicle
        * "lon_origin": A double, controls the starting longitude of the marine vehicle
        * "speed": 1.5: A double in metres, controls the speed of the marine vehicle

### Example configuration

'''
    {
        "lat_origin": 60.100863,
        "lon_origin": 19.921260,

        "wind_direction": 0,
        "wind_speed": 3,

        "boat_update_ms":100,
        "ais_update_ms":1000,

        "traffic": [
            {
                "mmsi": 100100100,
                "heading": 5,
                "lat_origin": 60.101184,
                "lon_origin": 19.920981,
                "speed": 2.2
            },
            {
                "mmsi": 100100101,
                "heading": 170,
                "lat_origin": 60.094990,
                "lon_origin": 19.921201,
                "speed": 1.5
            },
        ]
    }
'''