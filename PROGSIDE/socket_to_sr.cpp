#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>

#include "handler.h"
#include "cv7serial.h"
#include "maestro.h"
#include "nmea.h"
#include "ptmx.h"


//-----------------------------------------------------------------------------
struct PTMX ptmx_cv7;
struct PTMX ptmx_gps;
struct PTMX ptmx_maestro;
struct HANDLERS handler_shm;


//-----------------------------------------------------------------------------
void signals_handler(int signal_number)
{
    printf("Signal catched.\n");
    ptmxclose(&ptmx_cv7);
    ptmxclose(&ptmx_gps);
    ptmxclose(&ptmx_maestro);
    hndclose(&handler_shm);
    exit(EXIT_SUCCESS);
}

//-----------------------------------------------------------------------------
void exit_function(){
  printf("Closing virtual serial port and shared memory.\n");
  ptmxclose(&ptmx_cv7);
  ptmxclose(&ptmx_gps);
  ptmxclose(&ptmx_maestro);
  hndclose(&handler_shm);
}

int main(int argc,char **argv){

  // open virtual serial port
  if ( hndopen(&handler_shm) == -1)
  {
      fprintf(stderr, "Error on hndopen(): %s\n", strerror(errno));
      exit(EXIT_FAILURE);
  }

  // open virtual serial port
  if ( ptmxopen(&ptmx_cv7) == -1)
  {
      fprintf(stderr, "Error on ptmxopen() for cv7: %s\n", strerror(errno));
      exit(EXIT_FAILURE);
  }
  // print
  printf("PTTY cv7: %s\n", ptmx_cv7.port);

  // open virtual serial port
  if ( ptmxopen(&ptmx_gps) == -1)
  {
      fprintf(stderr, "Error on ptmxopen() for gps: %s\n", strerror(errno));
      exit(EXIT_FAILURE);
  }
  // print
  printf("PTTY gps: %s\n", ptmx_gps.port);

  // open virtual serial port
  if ( ptmxopen(&ptmx_maestro) == -1)
  {
      fprintf(stderr, "Error on ptmxopen() for maestro: %s\n", strerror(errno));
      exit(EXIT_FAILURE);
  }
  // print
  printf("PTTY maestro: %s\n", ptmx_maestro.port);

  // signals handler
  struct sigaction action;
  action.sa_handler = signals_handler;
  sigemptyset(& (action.sa_mask));
  action.sa_flags = 0;
  sigaction(SIGINT, & action, NULL);
  atexit(exit_function);


  return 0;
}
