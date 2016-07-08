all: init libptmx libnmea gps

init:
	mkdir -p bin

gps:
	cd src && make

libptmx: init
	cd ../library/ptmx && make

libnmea: init
	cd ../library/nmea && make

clean:
	find . -name *.o | xargs rm -f
	rm -rf bin

clean-lib:
	cd ../library && find . -name *.o | xargs rm -f

clean-all:clean clean-lib
