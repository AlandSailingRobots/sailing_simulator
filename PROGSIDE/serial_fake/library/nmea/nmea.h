#ifndef NMEA_H
#define NMEA_H

//-----------------------------------------------------------------------------
#define NMEA_LAT_SIZE 10
#define NMEA_LONG_SIZE 11
#define NMEA_DATE_SIZE 7
#define NMEA_DATE_RMC_SIZE 10
#define NMEA_COURSE_SIZE 6
#define NMEA_COURSE_DEG_SIZE 6
#define NMEA_SPEED_SIZE 6

#define NMEA_HEADER "$GP"
#define NMEA_SEPARATOR ','

#define NMEA_FRAME_GLL "GLL"
#define NMEA_GLL_SIZE 38

#define NMEA_FRAME_VTG "VTG"
#define NMEA_VTG_SIZE 39

#define NMEA_FRAME_RMC "RMC"
#define NMEA_RMC_SIZE 80

#define NMEA_FRAME_GSA "GSA"
#define NMEA_GSA_SIZE 39

//-----------------------------------------------------------------------------
struct NMEA_GLL
{
    char frame[NMEA_GLL_SIZE];
    float latitude; // decimal
    float longitude; // decimal
};

struct NMEA_VTG
{
    char frame[NMEA_VTG_SIZE];
    float course_real; // deg
    float course_magn; // deg
    float speed_knot;
    float speed_kmh;
};

struct NMEA_RMC
{
    char frame[NMEA_RMC_SIZE];
    float latitude; // decimal
    float longitude; // decimal
    float course_real; // deg
    float course_magn; // deg
    float speed_knot;
    float speed_kmh;
};

struct NMEA_GSA
{
    char frame[NMEA_GSA_SIZE];
    float course_real; // deg
    float course_magn; // deg
    float speed_knot;
    float speed_kmh;
};

//-----------------------------------------------------------------------------
#ifdef __cplusplus
extern "C" {
#endif

int nmea_gll(struct NMEA_GLL *gll);
int nmea_vtg(struct NMEA_VTG *vtg);
int nmea_rmc(struct NMEA_RMC *rmc);
int nmea_gsa(struct NMEA_GSA *gsa);

#ifdef __cplusplus
}
#endif


#endif
