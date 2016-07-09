INCLUDE_PATH=-I./serial_fake/include -I./i2c_fake/include
LIBS_I2C_PATH=./i2c_fake/lib
LIBS_SERIAL_PATH=./serial_fake/lib

OBJ_SER = maestro.o cv7serial.o nmea.o ptmx.o
OBJECT_SERIAL = $(addprefix $(LIBS_SERIAL_PATH)/, $(OBJ_SER))

OBJ_I2C = handler.o
OBJECT_I2C = $(addprefix $(LIBS_I2C_PATH)/, $(OBJ_I2C))

OBJECT = $(OBJECT_I2C) $(OBJECT_SERIAL)

SRC=socket_to_sr.cpp
EXECUTABLE=socket_to_sr

FLAG=-g -Wall -pedantic -Werror -std=c++11

all: libs $(EXECUTABLE)


$(EXECUTABLE):$(SRC) $(OBJECT)
	g++ $(FLAG) $(OBJECT) -o $(EXECUTABLE) $(SRC) $(INCLUDE_PATH) -lpthread -lrt

libs:
	cd i2c_fake/i2c_backend && make
	cd serial_fake/library && make


clean-all:
	cd i2c_fake/i2c_backend && make clean
	cd serial_fake/library && make clean
	rm $(EXECUTABLE)
