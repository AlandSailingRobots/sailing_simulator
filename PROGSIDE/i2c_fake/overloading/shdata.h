#ifndef SHDATA_H
#define SHDATA_H
#include <stdint.h>

#define NEW_DATA_FROM_SIM 1
#define NEW_DATA_FROM_PROG 2
#define DATA_HAS_BEEN_READ 0

// HMC6343 Registers
#define REG_SLAVE_ADDRESS 0x00

// HMC6343 Commands
#define COM_POST_HEADING 0x50
#define COM_POST_TILT 0x55
#define COM_POST_MAG 0x45
#define COM_POST_ACCEL 0x40
#define COM_READ_EEPROM 0xE1

struct SHDATA_ARDU
{
    uint8_t pressure_lsb;
    uint8_t rudder_lsb;
    uint8_t sheet_lsb;
    uint8_t battery_lsb;
    uint8_t pressure_msb;
    uint8_t rudder_msb;
    uint8_t sheet_msb;
    uint8_t battery_msb;

    uint8_t address_arduino;
    uint8_t flag;
};

struct SHDATA_COMP
{
    uint8_t headingVector[6];
    uint8_t magVector[6];
    uint8_t tiltVector[6];
    uint8_t accelVector[6];

    uint8_t address_compass;
    uint8_t flag;
};

struct SHDATA{
    struct SHDATA_ARDU shdata_arduino;
    struct SHDATA_COMP shdata_compass;
};

#endif // SHDATA_H
