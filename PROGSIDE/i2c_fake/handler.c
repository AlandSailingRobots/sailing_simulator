#include <stdlib.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/timerfd.h>
#include <signal.h>

#include "handler.h"

char *SHM_NAME = (char*)"i2c_shm_simu";
char *SEM_NAME = (char*)"i2c_sem_simu";

static struct HANDLERS *handlers;
static uint8_t readingProccess;
static uint8_t stepInReadingProcess; //for reading with more than one step;
static struct sigaction action;
static int initialized = 0;

//-----------------------------------------------------------------------------
void stopFunction(){
  hndclose();
  _Exit(2);
}


//-----------------------------------------------------------------------------
void signals_handler(int signal_number)
{
    printf("Signal catched.\n");
    hndclose();
    exit(EXIT_SUCCESS);
}

//-----------------------------------------------------------------------------
void hndinit()
{
    if (!initialized){
      handlers->shm = NULL;
      handlers->shmfd = -1;
      handlers->shdata = NULL;
      handlers->sem = NULL;
      readingProccess = 0;
      stepInReadingProcess = 0; //for reading with more than one step;
      // signals handler
      action.sa_handler = signals_handler;
      sigemptyset(& (action.sa_mask));
      action.sa_flags = 0;
      sigaction(SIGINT, & action, NULL);
      initialized = 1;
    }
}



//-----------------------------------------------------------------------------
int hndopen(int device)
{
    // init
    hndinit();

    int i = atexit(stopFunction);
    if (i != 0) {
              fprintf(stderr, "cannot set exit function\n");
              exit(EXIT_FAILURE);
    }

    // open semaphore
    handlers->sem = sem_open(SEM_NAME, O_RDWR);
    if (handlers->sem == SEM_FAILED)
    {
        perror ("sem_open");
        goto err;
    }

    // open shared memory and projection
    handlers->shm = SHM_NAME;
    handlers->shmfd = shm_open(SHM_NAME, O_RDWR, S_IRUSR|S_IWUSR);
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
    hndclose();
    return -1;
}

//-----------------------------------------------------------------------------
void hndclose()
{
    if(handlers->shmfd != -1)
        close(handlers->shmfd);

    if (handlers->sem != NULL)
        sem_close(handlers->sem);

    hndinit();
}

//-----------------------------------------------------------------------------
void readBlocks(uint8_t  *block){
    sem_wait(handlers->sem);
    block[2] = handlers->shdata->shdata_arduino->pressure_msb;
    block[3] = handlers->shdata->shdata_arduino->pressure_lsb;
    block[4] = handlers->shdata->shdata_arduino->rudder_msb;
    block[5] = handlers->shdata->shdata_arduino->rudder_lsb;
    block[6] = handlers->shdata->shdata_arduino->sheet_msb;
    block[7] = handlers->shdata->shdata_arduino->sheet_lsb;
    block[8] = handlers->shdata->shdata_arduino->battery_msb;
    block[9] = handlers->shdata->shdata_arduino->battery_lsb;
    block[10] = handlers->shdata->shdata_arduino->address_arduino;
    sem_post(handlers->sem);
}

//-----------------------------------------------------------------------------
int readOneCompassByte(){
   int return_value=0x00;
   sem_wait(handlers->sem);

   switch (readingProccess) {
     case COM_POST_HEADING:return_value = handlers->shdata->shdata_compass->headingVector[stepInReadingProcess++];
         break;
     case COM_POST_TILT:return_value = handlers->shdata->shdata_compass->tiltVector[stepInReadingProcess++];
         break;
     case COM_POST_MAG:return_value = handlers->shdata->shdata_compass->magVector[stepInReadingProcess++];
         break;
     case COM_POST_ACCEL:return_value = handlers->shdata->shdata_compass->accelVector[stepInReadingProcess++];
         break;
     case REG_SLAVE_ADDRESS:return_value = handlers->shdata->shdata_compass->address_compass;
         break;
     default: return_value = 0;
   }


   if (stepInReadingProcess>5)
       stepInReadingProcess=0;

   sem_wait(handlers->sem);
   return return_value;
}

//-----------------------------------------------------------------------------
void writeCommand(uint8_t data){
   readingProccess = data;
   //TODO check if changing 
}