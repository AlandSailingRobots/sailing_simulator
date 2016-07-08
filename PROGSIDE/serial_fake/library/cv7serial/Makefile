SONAME = libcv7serial.so
GCC = gcc

all:
	$(GCC) -c -fPIC cv7serial.c -o cv7serial.o
	$(GCC) -shared -Wl,-soname,$(SONAME) -o $(SONAME) cv7serial.o -lm
	cp cv7serial.h ../../include
	mv $(SONAME) ../../lib
