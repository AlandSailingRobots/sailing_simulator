SONAME = libmaestro.so
GCC = g++

all:
	$(GCC) -c -fPIC maestro.cpp -o maestro.o
	$(GCC) -shared -Wl,-soname,$(SONAME) -o $(SONAME) maestro.o
	cp maestro.h ../../include
	mv $(SONAME) ../../lib
