SRC = maestro
GCC = gcc

all:$(SRC).cpp
	$(GCC) -c -fPIC $(SRC).cpp -o $(SRC).o
	@mkdir -p ../../include
	@mkdir -p ../../lib
	@cp $(SRC).h ../../include
	@mv $(SRC).o ../../lib
