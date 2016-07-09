
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

all: $(TARGET_WIRO) handler.o

$(TARGET_WIRO): handler.o
			ar rcu myhook.a handler.o
			$(CC) -shared -Wall -Wl,-soname,$(TARGET_WIRO) -o $(TARGET_WIRO) -ldl -Wl,--whole-archive myhook.a -Wl,--no-whole-archive

handler.o: handler.c
			$(CC) -c $(FLAGS) handler.c -o handler.o

clean:
		rm -f *.so *.o *.a
