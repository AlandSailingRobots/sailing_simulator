#!/bin/bash


function clean_up {

	# Perform program exit housekeeping

  ps -ef | grep "/sr ./sailing_simulator/ simu_asr.db" | grep -v grep | awk '{print $2}' | xargs kill -9 --verbose

  sleep 3
  ps -ef | grep gpsd  | grep -v grep | awk '{print $2}' | xargs kill -9 --verbose
  ps -ef | grep socket_to_sr  | grep -v grep | awk '{print $2}' | xargs kill -s SIGINT --verbose
	exit
}



SIMU_PATH=./PROGSIDE

sh -c "$SIMU_PATH/socket_to_sr $SIMU_PATH/port_path.sh" &
sleep 1
source $SIMU_PATH/port_path.sh

echo $CV7_PORT_SIMU
echo $GPS_PORT_SIMU
echo $MAESTRO_PORT_SIMU

gpsd -N $GPS_PORT_SIMU &

sqlite3 simu_asr.db "UPDATE maestro_controller_config SET port='$MAESTRO_PORT_SIMU';"
sqlite3 simu_asr.db "UPDATE windsensor_config SET port='$CV7_PORT_SIMU';"

#python $SIMU_PATH/../SIMSIDE/python/simulation_main.py &

sleep 1

cd ../ && LD_PRELOAD=sailing_simulator/PROGSIDE/libwiringPiH.so ./sr ./sailing_simulator/simu_asr.db &

trap clean_up SIGINT SIGTERM SIGHUP
wait %1
