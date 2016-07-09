SRC = nmea
GCC = gcc

all:$(SRC).c
	$(GCC) -c -fPIC $(SRC).c -o $(SRC).o
	@mkdir -p ../../include
	@mkdir -p ../../lib
	@cp $(SRC).h ../../include
	@mv $(SRC).o ../../lib
