SAILING_SIMULATOR
=================

This repository need to be inside the [sailingrobot](https://github.com/AlandSailingRobots/sailingrobot) repository as a submodule:

Donwload it as a submodule in sailingrobot repository:

    $ git submodule init
    $ git submodule sync
    $ git submodule update

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

## Build the program side:

    $ cd PROGSIDE && make
    $ cd ..

## Create the simulation database

    $ ./simu_installdb.sh

You may want to change some value:

    $ ./simu_updateDB.sh

or (ie To change between waypoint following and line following):

    $ sqlite3 simu_asr.db
    $ update sailing_robot_config set line_follow=1; (*or 0 for waypoint routing*)

## Launch the simulation (in *sailingrobot/sailing_simulator*)

    $ ./simu_run.sh

You may want to launch the simulation on another computer than where the code is running,
you will need to comment the line 33 in *simu_run.sh* and change the ip address when launching the python:

    $ cd SIMSIDE/python
    $ ./simulation_main.py ip_address

It should be launch after socket_to_sr programme but preferably before sr (programm to test)
