

.PHONY: create_folder clean clean-unit clean-all

all: create_folder ../lib/handler.o

../lib/handler.o:
	gcc -c -fPIC handler.c  -o handler.o
	mv handler.o ../lib
	cp *.h ../include

i2c_backend: i2c_backend.c
	gcc -o i2c_backend i2c_backend.c handler.c -lpthread -lrt


create_folder:
	echo "Creation of folder"
	@mkdir -p ../include
	@mkdir -p ../lib

clean:
	@find ../ -name *.o | xargs rm -f
	@rm -r ../include
	@rm -r ../lib

clean-unit:
	@rm i2c_backend

clean-all: clean clean-unit
