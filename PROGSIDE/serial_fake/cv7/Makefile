all: init libptmx libcv7serial cv7

init:
	mkdir -p bin

cv7:
	cd src && make

libptmx: init
	cd ../library/ptmx && make

libcv7serial: init
	cd ../library/cv7serial && make

clean:
	find . -name *.o | xargs rm -f
	rm -rf bin

clean-lib:
	cd ../library && find . -name *.o | xargs rm -f

clean-all:clean clean-lib
