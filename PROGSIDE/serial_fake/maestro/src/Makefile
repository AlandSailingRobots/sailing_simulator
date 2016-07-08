GCC = gcc
BIN = cv7
TOP = ../..
TOPCV7 = ..

all:
	$(GCC) $(BIN).c -o $(BIN) -I $(TOP)/include -L $(TOP)/lib/ -lptmx -lcv7serial
	mv $(BIN) $(TOPCV7)/bin
