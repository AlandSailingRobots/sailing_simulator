SAILING_SIMULATOR
=================


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

or

    $ sqlite3 simu_asr.db

## Launch the simulation

This repository need to be inside the [sailingrobot](https://github.com/AlandSailingRobots/sailingrobot) repository as a submodule:

    $ ./simu_run.sh

This launch the sailingrobot side of the simulation, in another terminal:

    $ cd SIMSIDE/python
    $ python simulation_main.py
