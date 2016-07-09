#include <stdlib.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/timerfd.h>
#include <signal.h>

#include "handler.h"

static char *SHM_NAME = (char*)"i2c_shm_simu";
static char *SEM_NAME = (char*)"i2c_sem_simu";


//-----------------------------------------------------------------------------
void hndinit(struct HANDLERS *handlers)
{
      handlers->shm = NULL;
      handlers->shmfd = -1;
      handlers->shdata = NULL;
      handlers->sem = NULL;
}



//-----------------------------------------------------------------------------
int hndopen(struct HANDLERS *handlers)
{
    // init
    hndinit(handlers);

    // open semaphore
    handlers->sem = sem_open(SEM_NAME, O_RDWR|O_CREAT);
    if (handlers->sem == SEM_FAILED)
    {
        perror ("sem_open");
        goto err;
    }

    // open shared memory and projection
    handlers->shm = SHM_NAME;
    handlers->shmfd = shm_open(SHM_NAME, O_RDWR|O_CREAT, S_IRUSR|S_IWUSR);
    if (handlers->shmfd == -1)
    {
        perror("shm_open");
        goto err;
    }

    if (ftruncate(handlers->shmfd, sizeof(handlers->shdata)) != 0)
    {
        perror("ftruncate");
        goto err;
    }

    handlers->shdata = (struct SHDATA*) mmap(NULL, sizeof(handlers->shdata),
                            PROT_READ|PROT_WRITE, MAP_SHARED, handlers->shmfd, 0);
    if (handlers->shdata == MAP_FAILED)
    {
        perror("mmap");
        goto err;
    }

    return 0;

err:
    hndclose(handlers);
    return -1;
}

//-----------------------------------------------------------------------------
void hndclose(struct HANDLERS *handlers)
{
    if(handlers->shmfd != -1)
        close(handlers->shmfd);

    if (handlers->sem != NULL)
        sem_close(handlers->sem);

    hndinit(handlers);
}
