//-----------------------------------------------------------------------------
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sys/timerfd.h>

#include "i2c_backend/handler.h"

//-----------------------------------------------------------------------------
struct HANDLERS handlers;

//-----------------------------------------------------------------------------
void signals_handler(int signal_number)
{
    printf("Signal catched.\n");
    hndclose(&handlers);
    exit(EXIT_SUCCESS);
}

//-----------------------------------------------------------------------------
int write_handlers(struct HANDLERS *handlers)
{

    sem_wait(handlers->sem);
    handlers->shdata->shdata_arduino.battery_msb = 0;
    handlers->shdata->shdata_arduino.battery_lsb = 0;
    handlers->shdata->shdata_arduino.pressure_msb = 0;
    handlers->shdata->shdata_arduino.pressure_lsb += 1;
    handlers->shdata->shdata_arduino.rudder_msb = 0;
    handlers->shdata->shdata_arduino.rudder_lsb = 0;
    handlers->shdata->shdata_arduino.sheet_msb = 0;
    handlers->shdata->shdata_arduino.sheet_lsb = 0;
    handlers->shdata->shdata_arduino.flag = 0;
    handlers->shdata->shdata_compass.headingVector[1]+=1;
    sem_post(handlers->sem);
    return 1;
}


//-----------------------------------------------------------------------------
int main(int argv,char **argc)
{
    // open virtual serial port
    if ( hndopen(&handlers) == -1)
    {
        fprintf(stderr, "Error on hndopen(): %s\n", strerror(errno));
        exit(EXIT_FAILURE);
    }

    // signals handler
    struct sigaction action;
    action.sa_handler = signals_handler;
    sigemptyset(& (action.sa_mask));
    action.sa_flags = 0;
    sigaction(SIGINT, & action, NULL);

    printf("Waiting semaphore %s...\n",handlers.shm);
    fflush(stdout);
    sem_wait(handlers.sem);
    handlers.shdata->shdata_arduino.address_arduino = 0x07;
    handlers.shdata->shdata_arduino.battery_msb = 0;
    handlers.shdata->shdata_arduino.battery_lsb = 0;
    handlers.shdata->shdata_arduino.pressure_msb = 0;
    handlers.shdata->shdata_arduino.pressure_lsb = 0;
    handlers.shdata->shdata_arduino.rudder_msb = 0;
    handlers.shdata->shdata_arduino.rudder_lsb = 0;
    handlers.shdata->shdata_arduino.sheet_msb = 0;
    handlers.shdata->shdata_arduino.sheet_lsb = 0;
    handlers.shdata->shdata_arduino.flag = 0;
    memset(&handlers.shdata->shdata_compass,0,sizeof(struct SHDATA_COMP));
    handlers.shdata->shdata_compass.address_compass = 0x19;
    sem_post(handlers.sem);
    // print
    printf("SHM: %s\n", handlers.shm);
    fflush(stdout);
    // work
    while(1)
    {
        //sleep(2);
        write_handlers(&handlers);
        printf("loop\n");
        fflush(stdout);
        sleep(2);
    }

    hndclose(&handlers);

    return EXIT_SUCCESS;
}
