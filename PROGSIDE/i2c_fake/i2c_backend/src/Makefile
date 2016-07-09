GCC = gcc
BIN = i2c_backend
TOP = ../..
TOPGPS = ..

all:
	$(GCC) -g $(BIN).c handler.c -o $(BIN) -I ./  -lrt -lpthread

clean:
	rm i2c_backend
