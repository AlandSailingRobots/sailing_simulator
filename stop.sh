#!/bin/bash

ps -ef | grep "/sr ./sailing_simulator/simu_asr.db" | grep -v grep | awk '{print $2}' | xargs kill -s SIGINT --verbose
sleep 1
ps -ef | grep gpsd | grep -v grep | awk '{print $2}' | xargs kill -9 --verbose
ps -ef | grep socket_to_sr  | grep -v grep | awk '{print $2}' | xargs kill -s SIGINT --verbose
