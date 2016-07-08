#ifndef CV7SERIAL_H
#define CV7SERIAL_H

//-----------------------------------------------------------------------------
#define CV7PHRASE_SIZE 32

//---------------
struct CV7PHRASE
{
    char frame[CV7PHRASE_SIZE];
    float windDirection; // decimal
    float windSpeed; // decimal
    float windTemperature;
};

//-----------------------------------------------------------------------------
#ifdef __cplusplus
extern "C" {
#endif

int cv7_phrase(struct CV7PHRASE *cv7phrase);

#ifdef __cplusplus
}
#endif

#endif
