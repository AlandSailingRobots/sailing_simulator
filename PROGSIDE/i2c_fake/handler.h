#ifndef HANDLER_H
#define HANDLER_H

#include "shdata.h"
#include <semaphore.h>

//-----------------------------------------------------------------------------
struct HANDLERS
{
    sem_t * sem;
    char * shm;
    int shmfd;
    struct SHDATA *shdata;
};

//-----------------------------------------------------------------------------
int hndopen();
void hndclose();
void readBlocks(uint8_t  *block);
int readOneCompassByte();
void writeCommand(uint8_t data);

#endif // HANDLER_H
