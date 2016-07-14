#######################################################
#
#    Aland Sailing Robot
#    ===========================================
#    sailing_simulator
#    -------------------------------------------
#
#######################################################


# Directories
export BUILD_DIR = build
SRC_DIR = ./
OUTPUT_DIR = ./

NMEA= library/nmea/nmea.c
PTMX= library/ptmx/ptmx.c
CV7SERIAL= library/cv7serial/cv7serial.c
MAESTRO= library/maestro/maestro.cpp
I2C_BACKEND= library/i2c_backend/handler.c
OVERLOADING= library/overloading/handler.c

SRC_MAIN=socket_to_sr.cpp
SRC=thread_serial.cpp $(NMEA) $(PTMX) $(CV7SERIAL) $(MAESTRO) $(I2C_BACKEND)

HEADER_MAIN=socket_to_sr.hpp thread_serial.h

export INC=-I./include

# Object files
OBJECTS_C = $(addprefix $(BUILD_DIR)/, $(SRC:.c=.o))
OBJECTS_CPP = $(OBJECTS_C:.cpp=.o)
OBJECTS = $(OBJECTS_CPP)

OBJECT_OVERLOAD = $(BUILD_DIR)/library/overloading/libwiringPiH.so

OBJECT_MAIN = $(addprefix $(BUILD_DIR)/, $(SRC_MAIN:.cpp=.o))

TEST_MAKEFILE = ./Makefile.tests

# Target Output
EXECUTABLE = socket_to_sr
export OBJECT_FILE = $(BUILD_DIR)/objects.tmp

export CFLAGS = -Wall -g -o2
export CPPFLAGS = -g -Wall -pedantic -Werror -std=c++11
export LIBS = -lrt -lpthread -lm
CC = gcc
CXX = g++
SIZE = size

export CC
export CXX

export MKDIR_P = mkdir -p

#######################################################
# Rules
#######################################################

.PHONY: clean clean_tests

all: $(OBJECT_OVERLOAD) $(EXECUTABLE) stats

# Builds the intergration test, requires the whole system to be built before
build_tests: $(OBJECTS) $(EXECUTABLE)
	@echo Building tests...
	$(MAKE) -f $(TEST_MAKEFILE)

clean_tests:
	@echo Cleaning tests...
	@$(MAKE) -f $(TEST_MAKEFILE) clean

#  Create the directories needed
$(BUILD_DIR):
	@$(MKDIR_P) $(BUILD_DIR)

# Link and build
$(EXECUTABLE) : $(BUILD_DIR) $(OBJECTS) $(OBJECT_MAIN)
	rm -f $(OBJECT_FILE)
	@echo Linking object files
	@echo -n " " $(OBJECTS) >> $(OBJECT_FILE)
	$(CXX) $(LDFLAGS) @$(OBJECT_FILE) $(OBJECT_MAIN) -o $@ $(LIBS)

# Compile CPP files into the build folder
$(BUILD_DIR)/%.o:$(SRC_DIR)/%.cpp
	@mkdir -p $(dir $@)
	@echo Compiling CPP File: $@
	@$(CXX) -c $(CPPFLAGS) $(INC) -o ./$@ $<  $(LIBS)

 # Compile C files into the build folder
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c
	@mkdir -p $(dir $@)
	@echo Compiling C File: $@
	@$(CC) -c $(CFLAGS) $(INC) -o ./$@ $< $(LIBS)

#SPECIAL COMPILATION for overloading library
$(OBJECT_OVERLOAD): $(BUILD_DIR) $(SRC_DIR)/$(OVERLOADING)
	mkdir -p $(BUILD_DIR)/library/overloading
	@echo Compiling CPP File to Shared library:  $(dir $@)
	$(CXX) -c -fPIC $(CPPFLAGS) $(INC) $(SRC_DIR)$(OVERLOADING) -o $(BUILD_DIR)/$(OVERLOADING:.c=.o)
	$(CXX) -shared -Wall -Wl,-soname,$(OBJECT_OVERLOAD) -o $(OBJECT_OVERLOAD) -ldl  $(BUILD_DIR)/$(OVERLOADING:.c=.o)
	cp $(OBJECT_OVERLOAD) $(SRC_DIR)

#####################################################################
# Tool Rules

stats:$(EXECUTABLE)
	@echo Final executable size:
	$(SIZE) $(EXECUTABLE)

clean: clean_tests
	@echo Removing existing object files and executable
	@rm -f -r $(BUILD_DIR)
	@rm -f $(EXECUTABLE)
	@echo DONE
