SAILING_SIMULATOR
=================

This repository need to be inside the [sailingrobot](https://github.com/AlandSailingRobots/sailingrobot) repository as a submodule:

## Required Packages:

For python side of the simulation:

* python-numpy
* python-matplotlib
* [python-pip] optionnal

### How to get python packages:

On Arch linux as sudo:

    $ pacman -Syy && pacman -S python-pip
    $ pip install numpy
    $ pip install matplotlib

In some cases install for *matplotlib* is corrupted in that case you need to build it:

    $ pip uninstall matplotlib
    $ git clone https://github.com/matplotlib/matplotlib.git
    $ cd matplotlib
    $ python setup.py install

# Two way simulation:
There is now two way to use the simulator:

 * The compile time simulation
 * The post compile simulation

## Compile time simulation:

In repository [sailingrobot](https://github.com/AlandSailingRobots/sailingrobot):

    $ make [target] USE_SIM=1 (add -j for faster compilation)
    $ ./sr

Target is one of the boats:

  * target = ASPire/Janet

Here you will use the database in sailingrobot.
You may need to clean the build before making the executable:

    $ make clean

Then launch the python code [sailing_simulator/simulation_main.py](sailing_simulator/simulation_main.py):

    $ ./simulation_main.py [Boat] [AIS traffic]

The two arguments only accept 0 or 1 as its value:

  * Boat: 0 = simulating with Janet (Sail), 1 = ASPire (Wingsail)
  * AIS traffic: 0 = simulating without traffic, 1 = with traffic

## Post Compile time simulation

Download it as a submodule in sailingrobot repository:

    $ git submodule init
    $ git submodule sync
    $ git submodule update

### Build the program side:

    $ cd NavigationSystem && make [target] USE_SIM=1
    $ cd ../sailing_simulator


### Launch the simulation (in *sailingrobot/sailing_simulator*)

    $ ./simulation_main.py [Boat] [AIS traffic]

The two arguments only accept 0 or 1 as its value:

  * Boat: 0 = simulating with Janet (Sail), 1 = ASPire (Wingsail)
  * AIS traffic: 0 = simulating without traffic, 1 = with traffic
