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

    $ make USE_SIM=1 (add -j for faster compilation)
    $ ./sr

Here you will use the database in sailingrobot.
You may need to clean the build before making the executable:

    $ make clean

Then launch the python code [SIMSIDE/python/simulation_main.py](SIMSIDE/python/simulation_main.py):

    $ ./simulation_main.py [ip_address]

If on the same computer the IP address is optionnal (default is localhost).


## Post Compile time simulation

Download it as a submodule in sailingrobot repository:

    $ git submodule init
    $ git submodule sync
    $ git submodule update

### Build the program side:

    $ cd PROGSIDE && make
    $ cd ..

### Create the simulation database

    $ ./simu_installdb.sh

You may want to change some value:

    $ ./simu_updateDB.sh

or (ie To change between waypoint following and line following):

    $ sqlite3 simu_asr.db
    $ update sailing_robot_config set line_follow=1;

    (*or 0 for waypoint routing*)

### Launch the simulation (in *sailingrobot/sailing_simulator*)

    $ ./simu_run.sh

You may want to launch the simulation on another computer than where the code is running,
you will need to comment the line 33 in *simu_run.sh* and change the ip address when launching the python:

    $ cd SIMSIDE/python
    $ ./simulation_main.py ip_address
