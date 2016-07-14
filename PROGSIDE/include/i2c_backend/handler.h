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
#ifdef __cplusplus
extern "C" {
#endif

int hndopen(struct HANDLERS *handlers);
void hndclose(struct HANDLERS *handlers);

#ifdef __cplusplus
}
#endif

#endif // HANDLER_H
