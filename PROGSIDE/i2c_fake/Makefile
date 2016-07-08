
#######################################################
#
#    Aland Sailing Robot
#    ===========================================
#    MockI2C
#    -------------------------------------------
#
#######################################################

CC = g++
FLAGS =  -fPIC -g -Wall -pedantic -Werror -std=c++11
TARGET_WIRO = libwiringPiH.so


.PHONY: clean

all: $(TARGET_WIRO) wiringPi.o handler.o

$(TARGET_WIRO): wiringPi.o handler.o
			$(CC) -shared -Wl,-soname,$(TARGET_WIRO) -o $(TARGET_WIRO) wiringPi.o handler.o -ldl

wiringPi.o: wiringPi.c
			$(CC) -c $(FLAGS) wiringPi.c -o wiringPi.o

handler.o: handler.c
			$(CC) -c $(FLAGS) handler.c -o handler.o

clean:
		rm -f *.so *.o
