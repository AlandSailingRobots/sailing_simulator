GCC = gcc
BIN = gps
TOP = ../..
TOPGPS = ..

all:
	$(GCC) $(BIN).c -o $(BIN) -I $(TOP)/include -L $(TOP)/lib/ -lptmx -lnmea
	mv $(BIN) $(TOPGPS)/bin
