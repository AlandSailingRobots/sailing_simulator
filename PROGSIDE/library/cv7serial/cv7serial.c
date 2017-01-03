#include <string.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <stdlib.h>

#include "cv7serial.h"

#define KNOT_TO_KMH 1.852

//-----------------------------------------------------------------------------
int cv7_phrase(struct CV7PHRASE *cv7phrase)
{
    // set to zero
    memset(&cv7phrase->frame[0], 0, CV7PHRASE_SIZE);

    sprintf(cv7phrase->frame,"$IIMWV,%03.1f,R,%03.1f,M,A*3B\n$WIXDR,C,%03.1f,C,,*58\n",
      cv7phrase->windDirection,cv7phrase->windSpeed,cv7phrase->windTemperature);
    return 0;
}
