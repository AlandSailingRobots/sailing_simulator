#include <string.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <stdlib.h>

#include "nmea.h"

#define KNOT_TO_KMH 1.852

int iteration = 0;

//-----------------------------------------------------------------------------

int checksum(const char *s) {
    int c = 0;

    while(*s)
        c ^= *s++;

    return c;
}

//-----------------------------------------------------------------------------
int knot_to_kmh_str(float knot, size_t size, char * format, char * kmh_str)
{
    float kmh = KNOT_TO_KMH * knot;
    snprintf(kmh_str, size, format, kmh);

    return kmh;
}

//-----------------------------------------------------------------------------
int current_date_str(char * str)
{
    time_t rawtime;
    struct tm * timeinfo;

    time ( &rawtime );
    timeinfo = localtime ( &rawtime );

    sprintf(str, "%02d%02d%02d", timeinfo->tm_hour, timeinfo->tm_min,
            timeinfo->tm_sec);
    return 0;
}

int current_date_str_rmc(char * str)
{
    time_t rawtime;
    struct tm * timeinfo;

    time ( &rawtime );
    timeinfo = localtime ( &rawtime );

    sprintf(str, "%02d%02d%02d.00", timeinfo->tm_hour, timeinfo->tm_min,
            timeinfo->tm_sec);
    return 0;
}

int current_date_day(char * str)
{
    time_t rawtime;
    struct tm * timeinfo;

    time ( &rawtime );
    timeinfo = localtime ( &rawtime );

    sprintf(str, "%02d%02d%02d", timeinfo->tm_mday, timeinfo->tm_mon+1,
            timeinfo->tm_year-100);
    return 0;
}


//-----------------------------------------------------------------------------
int decimal_to_str(float decimal, size_t size, char * format, char *str)
{
    int deg = floor(decimal);
    float min = (decimal - deg) * 60;
    float coord = deg*100 + min;

    snprintf(str, size, format, coord);

    return coord;
}

//-----------------------------------------------------------------------------
int nmea_gll(struct NMEA_GLL *gll)
{
    // set to zero
    memset(&gll->frame[0], 0, NMEA_GLL_SIZE);

    // header
    memcpy(&gll->frame[0], NMEA_HEADER, 3);

    // frame name
    memcpy(&gll->frame[3], NMEA_FRAME_GLL, 3);

    // separator
    gll->frame[6] = NMEA_SEPARATOR;

    // latitude
    char lat_str[NMEA_LAT_SIZE];
    float lat = decimal_to_str(gll->latitude, NMEA_LAT_SIZE, "%07.2f", lat_str);
    memcpy(&gll->frame[7], lat_str, NMEA_LAT_SIZE);

    // separator
    gll->frame[14] = NMEA_SEPARATOR;

    // latitude orientation
    if (lat < 0)
        gll->frame[15] = 'S';
    else
        gll->frame[15] = 'N';

    // separator
    gll->frame[16] = NMEA_SEPARATOR;

    // longitude
    char long_str[NMEA_LAT_SIZE];
    float lon = decimal_to_str(gll->longitude, NMEA_LONG_SIZE, "%08.2f",
                                long_str);
    memcpy(&gll->frame[17], long_str, NMEA_LONG_SIZE);

    // separator
    gll->frame[25] = NMEA_SEPARATOR;

    // longitude orientation
    if (lon < 0)
        gll->frame[26] = 'W';
    else
        gll->frame[26] = 'E';

    // separator
    gll->frame[27] = NMEA_SEPARATOR;

    // date
    char date_str[NMEA_DATE_SIZE];
    current_date_str(date_str);
    memcpy(&gll->frame[28], date_str, NMEA_DATE_SIZE);

    // separator
    gll->frame[34] = NMEA_SEPARATOR;

    // valid
    gll->frame[35] = 'A';
    gll->frame[36] = '\n';

    return 0;
}

//-----------------------------------------------------------------------------
int nmea_vtg(struct NMEA_VTG *vtg)
{
    // set to zero
    memset(&vtg->frame[0], 0, NMEA_VTG_SIZE);

    // header
    memcpy(&vtg->frame[0], NMEA_HEADER, 3);

    // frame name
    memcpy(&vtg->frame[3], NMEA_FRAME_VTG, 3);

    // separator
    vtg->frame[6] = NMEA_SEPARATOR;

    // course real
    char course_real_str[NMEA_COURSE_SIZE];
    snprintf(course_real_str, NMEA_COURSE_SIZE, "%05.1f", vtg->course_real);
    memcpy(&vtg->frame[7], course_real_str, NMEA_COURSE_SIZE);

    // separator
    vtg->frame[12] = NMEA_SEPARATOR;

    // true track
    vtg->frame[13] = 'T';

    // separator
    vtg->frame[14] = NMEA_SEPARATOR;

    // course magnetic
    char course_magn_str[NMEA_COURSE_SIZE];
    snprintf(course_magn_str, NMEA_COURSE_SIZE, "%05.1f", vtg->course_magn);
    memcpy(&vtg->frame[15], course_magn_str, NMEA_COURSE_SIZE);

    // separator
    vtg->frame[20] = NMEA_SEPARATOR;

    // true track
    vtg->frame[21] = 'M';

    // separator
    vtg->frame[22] = NMEA_SEPARATOR;

    // not speed
    char speed_knot_str[NMEA_SPEED_SIZE];
    snprintf(speed_knot_str, NMEA_SPEED_SIZE, "%05.1f", vtg->speed_knot);
    memcpy(&vtg->frame[23], speed_knot_str, NMEA_SPEED_SIZE);

    // separator
    vtg->frame[28] = NMEA_SEPARATOR;

    // true track
    vtg->frame[29] = 'N';

    // separator
    vtg->frame[28] = NMEA_SEPARATOR;

    // kmh speed
    char speed_kmh_str[NMEA_SPEED_SIZE];
    knot_to_kmh_str(vtg->speed_knot, NMEA_SPEED_SIZE, "%05.1f", speed_kmh_str);
    memcpy(&vtg->frame[29], speed_kmh_str, NMEA_SPEED_SIZE);

    // separator
    vtg->frame[34] = NMEA_SEPARATOR;

    // true track
    vtg->frame[35] = 'K';
    vtg->frame[36] = '\n';

    return 0;
}

int nmea_gsa(struct NMEA_GSA *gsa)
{
    // set to zero
    memset(&gsa->frame[0], 0, NMEA_GSA_SIZE);

    // header
    memcpy(&gsa->frame[0], NMEA_HEADER, 3);

    // frame name
    memcpy(&gsa->frame[3], NMEA_FRAME_GSA, 3);

    // separator
    gsa->frame[6] = NMEA_SEPARATOR;

    // course real
    char course_real_str[NMEA_COURSE_SIZE];
    snprintf(course_real_str, NMEA_COURSE_SIZE, "%05.1f", gsa->course_real);
    memcpy(&gsa->frame[7], course_real_str, NMEA_COURSE_SIZE);

    // separator
    gsa->frame[12] = NMEA_SEPARATOR;

    // true track
    gsa->frame[13] = 'T';

    // separator
    gsa->frame[14] = NMEA_SEPARATOR;

    // course magnetic
    char course_magn_str[NMEA_COURSE_SIZE];
    snprintf(course_magn_str, NMEA_COURSE_SIZE, "%05.1f", gsa->course_magn);
    memcpy(&gsa->frame[15], course_magn_str, NMEA_COURSE_SIZE);

    // separator
    gsa->frame[20] = NMEA_SEPARATOR;

    // true track
    gsa->frame[21] = 'M';

    // separator
    gsa->frame[22] = NMEA_SEPARATOR;

    // not speed
    char speed_knot_str[NMEA_SPEED_SIZE];
    snprintf(speed_knot_str, NMEA_SPEED_SIZE, "%05.1f", gsa->speed_knot);
    memcpy(&gsa->frame[23], speed_knot_str, NMEA_SPEED_SIZE);

    // separator
    gsa->frame[28] = NMEA_SEPARATOR;

    // true track
    gsa->frame[29] = 'N';

    // separator
    gsa->frame[28] = NMEA_SEPARATOR;

    // kmh speed
    char speed_kmh_str[NMEA_SPEED_SIZE];
    knot_to_kmh_str(gsa->speed_knot, NMEA_SPEED_SIZE, "%05.1f", speed_kmh_str);
    memcpy(&gsa->frame[29], speed_kmh_str, NMEA_SPEED_SIZE);

    // separator
    gsa->frame[34] = NMEA_SEPARATOR;

    // true track
    gsa->frame[35] = 'K';
    gsa->frame[36] = '\n';

    return 0;
}

int nmea_rmc(struct NMEA_RMC *rmc)
{

    // set to zero
    memset(&rmc->frame[0], 0, NMEA_RMC_SIZE);

    // header
    memcpy(&rmc->frame[0], NMEA_HEADER, 3);

    // frame name
    memcpy(&rmc->frame[3], NMEA_FRAME_RMC, 3);

    // separator
    rmc->frame[6] = NMEA_SEPARATOR;

    //utc time
    char date_str[NMEA_DATE_RMC_SIZE];
    current_date_str_rmc(date_str);
    memcpy(&rmc->frame[7], date_str, NMEA_DATE_RMC_SIZE);

    // separator
    rmc->frame[16] = NMEA_SEPARATOR;

    //status
    rmc->frame[17] = 'A';

    // separator
    rmc->frame[18] = NMEA_SEPARATOR;

    // latitude
    char lat_str[NMEA_LAT_SIZE];
    float lat = decimal_to_str(rmc->latitude, NMEA_LAT_SIZE, "%09.4f", lat_str);
    memcpy(&rmc->frame[19], lat_str, NMEA_LAT_SIZE);

    // separator
    rmc->frame[28] = NMEA_SEPARATOR;

    // latitude orientation
    if (lat < 0)
        rmc->frame[29] = 'S';
    else
        rmc->frame[29] = 'N';

    // separator
    rmc->frame[30] = NMEA_SEPARATOR;

    // longitude
    char long_str[NMEA_LONG_SIZE];
    float lon = decimal_to_str(rmc->longitude, NMEA_LONG_SIZE, "%010.4f",
                                long_str);
    memcpy(&rmc->frame[31], long_str, NMEA_LONG_SIZE);

    // separator
    rmc->frame[41] = NMEA_SEPARATOR;

    // longitude orientation
    if (lon < 0)
        rmc->frame[42] = 'W';
    else
        rmc->frame[42] = 'E';

    // separator
    rmc->frame[43] = NMEA_SEPARATOR;

    // knot speed
    char speed_knot_str[NMEA_SPEED_SIZE];
    snprintf(speed_knot_str, NMEA_SPEED_SIZE, "%05.1f", rmc->speed_knot);
    memcpy(&rmc->frame[44], speed_knot_str, NMEA_SPEED_SIZE);

    // separator
    rmc->frame[49] = NMEA_SEPARATOR;



    // course gps
    char course_gps_str[NMEA_COURSE_DEG_SIZE];
    snprintf(course_gps_str, NMEA_COURSE_DEG_SIZE, "%05.1f", rmc->course_real);
    memcpy(&rmc->frame[50], course_gps_str, NMEA_COURSE_DEG_SIZE);
    // separator
    rmc->frame[55] = NMEA_SEPARATOR;

    // true track
    char date_day_str[NMEA_DATE_SIZE];
    current_date_day(date_day_str);
    memcpy(&rmc->frame[56], date_day_str, 6);

    // separator
    rmc->frame[62] = NMEA_SEPARATOR;


    // magnetic variation
    memcpy(&rmc->frame[63], "0.0", 3);

    // separator
    rmc->frame[66] = NMEA_SEPARATOR;

    //magnetic variation direction
    rmc->frame[67] = 'E';

    // separator
    rmc->frame[68] = NMEA_SEPARATOR;

    rmc->frame[69] = 'A';


    int check = checksum(rmc->frame);
    //checksum
//    rmc->frame[65] = '*';
    char checkcar[3] ={'\0','\0','\0'} ;
    sprintf(checkcar,"%02X",check);
//    memcpy(&rmc->frame[66], checkcar, 2);

    rmc->frame[70] = '\n';

    return 0;
}
