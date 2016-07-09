.PHONY: create_folder

all: create_folder ../lib/cv7serial.o ../lib/maestro.o ../lib/nmea.o ../lib/ptmx.o

../lib/cv7serial.o:
	@cd cv7serial && make

../lib/maestro.o:
	@cd maestro && make

../lib/nmea.o:
	@cd nmea && make

../lib/ptmx.o:
	@cd ptmx && make

create_folder.o:
	echo "Creation of folder"
	@mkdir -p ../include
	@mkdir -p ../lib

clean:
	@find ../ -name *.o | xargs rm -f
	@rm -r ../include
	@rm -r ../lib
