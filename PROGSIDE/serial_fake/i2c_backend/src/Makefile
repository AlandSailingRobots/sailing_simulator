GCC = gcc
BIN = i2c_backend
TOP = ../..
TOPGPS = ..

all:
	$(GCC) $(BIN).c -o $(BIN) -I $(TOP)/include -L $(TOP)/lib/ -lptmx -lnmea
	mv $(BIN) $(TOPGPS)/bin
