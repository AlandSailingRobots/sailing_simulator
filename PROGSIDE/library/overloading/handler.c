#include <stdlib.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/timerfd.h>
#include <signal.h>
#include <semaphore.h>

#include "shdata.h"


//-----------------------------------------------------------------------------
struct HANDLERS
{
    sem_t * sem;
    char * shm;
    int shmfd;
    struct SHDATA *shdata;
};

//-----------------------------------------------------------------------------
static int hndopen();
static void hndclose();
static int readBlocks(uint8_t  *block,int length);
static int readOneCompassByte();
static void writeCommand(uint8_t data);

static char *SHM_NAME = (char*)"/i2c_shm_simu";
static char *SEM_NAME = (char*)"/i2c_sem_simu";

static struct HANDLERS handlers;
static uint8_t readingProccess;
static uint8_t stepInReadingProcess; //for reading with more than one step;
static struct sigaction action;
static int initialized = 0;

//-----------------------------------------------------------------------------
static void stopFunction(){
  hndclose();
  _Exit(2);
}


//-----------------------------------------------------------------------------
static void signals_handler(int signal_number)
{
    printf("Signal catched.\n");
    hndclose();
    exit(EXIT_SUCCESS);
}

//-----------------------------------------------------------------------------
static void hndinit()
{
      handlers.shm = NULL;
      handlers.shmfd = -1;
      handlers.shdata = NULL;
      handlers.sem = NULL;
      readingProccess = 0;
      stepInReadingProcess = 0; //for reading with more than one step;
      // signals handler
      action.sa_handler = signals_handler;
      sigemptyset(& (action.sa_mask));
      action.sa_flags = 0;
      sigaction(SIGINT, & action, NULL);
      int i = atexit(stopFunction);
      if (i != 0) {
                fprintf(stderr, "cannot set exit function\n");
                exit(EXIT_FAILURE);
      }
}



//-----------------------------------------------------------------------------
static int hndopen()
{
    if (initialized)
        return 0;
    // init
    hndinit();

    // open semaphore
    handlers.sem = sem_open(SEM_NAME, O_RDWR);
    if (handlers.sem == SEM_FAILED)
    {
        perror ("sem_open");
        goto err;
    }

    // open shared memory and projection
    handlers.shm = SHM_NAME;
    handlers.shmfd = shm_open(SHM_NAME, O_RDWR, S_IRUSR|S_IWUSR);
    if (handlers.shmfd == -1)
    {
        perror("shm_open");
        goto err;
    }

    if (ftruncate(handlers.shmfd, sizeof(handlers.shdata)) != 0)
    {
        perror("ftruncate");
        goto err;
    }

    handlers.shdata = (struct SHDATA*) mmap(NULL, sizeof(handlers.shdata),
                            PROT_READ|PROT_WRITE, MAP_SHARED, handlers.shmfd, 0);
    if (handlers.shdata == MAP_FAILED)
    {
        perror("mmap");
        goto err;
    }
    initialized = 1;
    return 0;

err:
    hndclose();
    return -1;
}

//-----------------------------------------------------------------------------
static void hndclose()
{
    if(handlers.shmfd != -1)
        close(handlers.shmfd);

    if (handlers.sem != NULL)
        sem_close(handlers.sem);

    handlers.shm = NULL;
    handlers.shmfd = -1;
    handlers.shdata = NULL;
    handlers.sem = NULL;
    readingProccess = 0;
    stepInReadingProcess = 0; //for reading with more than one step;
    initialized = 0;
}

//-----------------------------------------------------------------------------
static int readBlocks(uint8_t  *block,int length){
    if (length < 9)
    {
      return 0;
    }
    sem_wait(handlers.sem);
    block[0] = handlers.shdata->shdata_arduino.pressure_msb;
    block[1] = handlers.shdata->shdata_arduino.pressure_lsb;
    block[2] = handlers.shdata->shdata_arduino.rudder_msb;
    block[3] = handlers.shdata->shdata_arduino.rudder_lsb;
    block[4] = handlers.shdata->shdata_arduino.sheet_msb;
    block[5] = handlers.shdata->shdata_arduino.sheet_lsb;
    block[6] = handlers.shdata->shdata_arduino.battery_msb;
    block[7] = handlers.shdata->shdata_arduino.battery_lsb;
    block[8] = handlers.shdata->shdata_arduino.address_arduino;
    sem_post(handlers.sem);
    return 9;
}

//-----------------------------------------------------------------------------
static int readOneCompassByte(){
   int return_value=0x00;
   sem_wait(handlers.sem);

   switch (readingProccess) {
     case COM_POST_HEADING:return_value = handlers.shdata->shdata_compass.headingVector[stepInReadingProcess++];
         break;
     case COM_POST_TILT:return_value = handlers.shdata->shdata_compass.tiltVector[stepInReadingProcess++];
         break;
     case COM_POST_MAG:return_value = handlers.shdata->shdata_compass.magVector[stepInReadingProcess++];
         break;
     case COM_POST_ACCEL:return_value = handlers.shdata->shdata_compass.accelVector[stepInReadingProcess++];
         break;
     case REG_SLAVE_ADDRESS:return_value = handlers.shdata->shdata_compass.address_compass;
         break;
     default: return_value = 0;
   }


   if (stepInReadingProcess>5)
       stepInReadingProcess=0;

   sem_post(handlers.sem);
   return return_value;
}



//-----------------------------------------------------------------------------
static void writeCommand(uint8_t data){
   readingProccess = data;
   //TODO check if changing
}


extern "C" {

int wiringPiI2CSetup(const int m_address){
  return hndopen();
}

int wiringPiI2CRead(int m_fd){
  //ONLY USED BY COMPASS DEVICE;
  return readOneCompassByte();
}

int wiringPiI2CWrite (int fd, int data){
  //ONLY USED BY COMPASS DEVICE
  writeCommand(data);
  return 1;
}



int wiringPiI2CReadBlock(int fd,uint8_t  *block,int length)
{
  return readBlocks(block,length);

}

}
