SAILING_SIMULATOR
=================

This repository Aland sailing robot's simulator. The simulator will simulate a sailing ASV and provides a basic simulation of other marine traffic. The GPS tracks of the ASV and the simulated track are outputted in .track files, which the first one being the ASV.

## Usage

    ./simulation_main.py [config] [AIS traffic]

Arguements:
* `[config]` - Optional:  This is the path to the simulation config file, if no path is included, the simulator will default to `Simu_config_0.json`
* `[AIS traffic]` - Optional : 0 = simulating without traffic, 1 = with traffic (default)

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
    $ pip install objgraph
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

    $ make [target] USE_SIM=1 (add -j for faster compilation)
    $ ./sr

Target is one of the boats:

  * `target = ASPire/Janet`

Here you will use the database in sailingrobot.
You may need to clean the build before making the executable:

    $ make clean

Then launch the python code [sailing_simulator/simulation_main.py](sailing_simulator/simulation_main.py):

    $ ./simulation_main.py [config] [AIS traffic]

* `[config]` - Optional:  This is the path to the simulation config file, if no path is included, the simulator will default to Simu_config_0.json
* `[AIS traffic]`: 0 = simulating without traffic, 1 = with traffic


## Simulator Configuration

The configuration file set the ASV configuration, the starting position of the ASV, the simulator frequency, as well as the sending frequency of the messages and simulated traffic.

### ASV Configuration file

* `"boat_config"`: Path to the boat configuration file.  
The boat configuration file set the type and the physics parameters of the simulated boat.
  * `"boat_type"`: 0 = simulating with Janet (Sail), 1 = ASPire (Wingsail)
  * ...

### ASV State

The ASV's initial state is controlled by 4 configurable variables

* `"lat_origin"` : A double, controls the starting latitude
* `"lon_origin"`: A double, controls the starting longitude
* `"wind_direction"`: A integer in degrees, controls the wind direction
* `"wind_speed"`: A integer in metres, controls the winds speed

### Message Updates

* `"boat_update_ms"`: A integer in milliseconds, controls how often boat state messages are sent out
* `"ais_update_ms"`: A integer in milliseconds, controls how often AIS contact messages are sent out

### Simulated Marine traffic

Simulated marine traffic can be defined in the configuration message, with the json array type with the key "traffic". Each entry describes a single simulated marine vehicle. 

A single marine vehicle is defined as:

* `"mmsi"`: The id of the vessel, an integer, used to uniquely id simulated traffic. An id below 100000000 will be regarded as a thermal imaging contact and not a AIS contact.
* `"heading"`: An integer, in degress, the heading of the vessel
* `"lat_origin"`: A double, controls the starting latitude of the marine vehicle
* `"lon_origin"`: A double, controls the starting longitude of the marine vehicle
* `"speed"`: 1.5: A double in metres, controls the speed of the marine vehicle

### Example configuration

```
    {
    "boat_config": "Janet_config.json",

    "simulation_step": 0.01,

    "lat_origin": 60.107240,
    "lon_origin": 19.922397,

    "wind_direction": 90,
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
```


