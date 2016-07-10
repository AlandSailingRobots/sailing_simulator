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

#include "socket_to_sr.hpp"

#define MAX(a,b) ((a) > (b) ? a : b)

//-----------------------------------------------------------------------------
struct PTMX ptmx_cv7;
struct PTMX ptmx_gps;
struct PTMX ptmx_maestro;
struct HANDLERS handler_shm;
struct THREAD_HANDLER handler_thread_gps;
struct THREAD_HANDLER handler_thread_cv7;
struct THREAD_HANDLER handler_thread_maestro;

struct DATA_SOCKET_SEND *data_socket_send;
struct DATA_SOCKET_RECEIVE *data_socket_receive;

  // init pthread
pthread_t thread_gps;
pthread_t thread_cv7;
pthread_t thread_maestro;



int *run_threads;
int maestro_pipe[2];

//-----------------------------------------------------------------------------
void exit_function(){

  printf("Closing threads.\n");
  char *c={'\0'};
  write(maestro_pipe[1],&c,1);
  pthread_mutex_lock(handler_thread_gps.mutex);
  *run_threads = 0;
  pthread_mutex_unlock(handler_thread_gps.mutex);

  printf("Waiting threads... \n");
  pthread_join(thread_gps, NULL);
  pthread_join(thread_maestro, NULL);
  pthread_join(thread_cv7, NULL);

  printf("Closing virtual serial port and shared memory.\n");
  ptmxclose(&ptmx_cv7);
  ptmxclose(&ptmx_gps);
  ptmxclose(&ptmx_maestro);
  hndclose(&handler_shm);
  if (run_threads!=NULL)
      free(run_threads);
  if (data_socket_send!=NULL)
      free(data_socket_send);
  if (data_socket_receive!=NULL)
      free(data_socket_receive);
}

//-----------------------------------------------------------------------------
void signals_handler(int signal_number)
{
    exit_function();
    _Exit(EXIT_SUCCESS);
}

int write_cv7(int fd,struct THREAD_HANDLER *th_handler)
{
    size_t bytes = 0;

    struct CV7PHRASE cv7phrase;
    pthread_mutex_lock(th_handler->mutex);
    cv7phrase.windDirection = th_handler->data_socket_receive->windDirection;
    cv7phrase.windSpeed = th_handler->data_socket_receive->windSpeed;
    cv7phrase.windTemperature = th_handler->data_socket_receive->windTemperature;
    pthread_mutex_unlock(th_handler->mutex);

    if ( cv7_phrase(&cv7phrase) == -1)
    {
        perror("cv7_phrase");
        exit(EXIT_FAILURE);
    }
    else
        bytes = write(fd, cv7phrase.frame, sizeof(cv7phrase.frame));

    return bytes;
}

int write_vtg(int fd,struct THREAD_HANDLER *th_handler)
{
    size_t bytes = 0;

    struct NMEA_VTG vtg;

    pthread_mutex_lock(th_handler->mutex);
    vtg.course_real = th_handler->data_socket_receive->course_real;
    vtg.course_magn = th_handler->data_socket_receive->course_magn;
    vtg.speed_knot = th_handler->data_socket_receive->speed_knot;
    pthread_mutex_unlock(th_handler->mutex);


    if ( nmea_vtg(&vtg) == -1)
    {
        perror("nmea_vtg");
        exit(EXIT_FAILURE);
    }
    else
    /*{
        bytes = write(fd, vtg.frame, sizeof(vtg.frame));
    }*/

    return bytes;
}

//-----------------------------------------------------------------------------
int write_gll(int fd,struct THREAD_HANDLER *th_handler)
{
    size_t bytes = 0;

    struct NMEA_GLL gll;

    pthread_mutex_lock(th_handler->mutex);
    gll.latitude = th_handler->data_socket_receive->latitude;
    gll.longitude = th_handler->data_socket_receive->longitude;
    pthread_mutex_unlock(th_handler->mutex);

    if ( nmea_gll(&gll) == -1)
    {
        perror("nmea_gll");
        exit(EXIT_FAILURE);
    }
    /*else
    {
        bytes = write(fd, gll.frame, sizeof(gll.frame));
    }*/

    return bytes;
}


int write_rmc(int fd,struct THREAD_HANDLER *th_handler)
{
    size_t bytes = 0;

    struct NMEA_RMC rmc;

    pthread_mutex_lock(th_handler->mutex);
    rmc.latitude = th_handler->data_socket_receive->latitude;
    rmc.longitude = th_handler->data_socket_receive->longitude;
    rmc.course_real = th_handler->data_socket_receive->course_real;
    rmc.course_magn = th_handler->data_socket_receive->course_magn;
    rmc.speed_knot = th_handler->data_socket_receive->speed_knot;
    pthread_mutex_unlock(th_handler->mutex);

    if ( nmea_rmc(&rmc) == -1)
    {
        perror("nmea_rmc");
        exit(EXIT_FAILURE);
    }
    else
    {
        bytes = write(fd, rmc.frame, sizeof(rmc.frame));
    }

    return bytes;
}

void *gps_thread(void *thread_handler){
  int run_thread;
  struct THREAD_HANDLER *th_handler = (struct THREAD_HANDLER*) thread_handler;
  pthread_mutex_lock(th_handler->mutex);
  run_thread = *(th_handler->run_threads);
  pthread_mutex_unlock(th_handler->mutex);

  while(run_thread){
    pthread_mutex_lock(th_handler->mutex);
    run_thread = *(th_handler->run_threads);
    pthread_mutex_unlock(th_handler->mutex);
    write_rmc(ptmx_gps.fd, th_handler);
    usleep(500000);
  }

  printf("GPS exiting\n");
  fflush(stdout);
  return NULL;
}

void *cv7_thread(void *thread_handler){
  int run_thread;
  struct THREAD_HANDLER *th_handler = (struct THREAD_HANDLER*) thread_handler;
  pthread_mutex_lock(th_handler->mutex);
  run_thread = *(th_handler->run_threads);
  pthread_mutex_unlock(th_handler->mutex);

  while(run_thread){
    pthread_mutex_lock(th_handler->mutex);
    run_thread = *(th_handler->run_threads);
    pthread_mutex_unlock(th_handler->mutex);
    write_cv7(ptmx_cv7.fd, th_handler);
    usleep(500000);
  }

  printf("CV7 exiting\n");
  fflush(stdout);
  return NULL;
}

void *maestro_thread(void *thread_handler){
  int run_thread;
  fd_set pdset;

  MaestroExchange maestro_handler;
  struct THREAD_HANDLER *th_handler = (struct THREAD_HANDLER*) thread_handler;
  pthread_mutex_lock(th_handler->mutex);
  run_thread = *(th_handler->run_threads);
  pthread_mutex_unlock(th_handler->mutex);

  while(run_thread){

    FD_ZERO(&pdset);
    FD_SET(ptmx_maestro.fd, &pdset);
    FD_SET(maestro_pipe[0], &pdset);
    int max = MAX(ptmx_maestro.fd,maestro_pipe[0]);
    select(max+1, &pdset, NULL, NULL, NULL);

    if (FD_ISSET(maestro_pipe[0], &pdset))
    {
     break;
    }
    char buffer[200];
    memset(buffer, '\0',200);
    int n = read(ptmx_maestro.fd, buffer, sizeof(buffer));
    if (n < 0)
      fputs("read failed!\n", stderr);
    else
      maestro_handler.add_buffer(buffer);

    pthread_mutex_lock(th_handler->mutex);
    run_thread = *(th_handler->run_threads);
    th_handler->data_socket_send->rudder_command = maestro_handler.get_rudder_command();
    th_handler->data_socket_send->sheet_command = maestro_handler.get_sheet_command();
    pthread_mutex_unlock(th_handler->mutex);
  }

  printf("MAESTRO exiting\n");
  fflush(stdout);
  return NULL;
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

  //init pipe for maestro
  pipe(maestro_pipe);

  // init mutex
  pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

  run_threads = (int*) malloc(sizeof(int));
  *run_threads = 1;

  //
  data_socket_send = (struct DATA_SOCKET_SEND*) malloc(sizeof(struct DATA_SOCKET_SEND));
  data_socket_receive = (struct DATA_SOCKET_RECEIVE*) malloc(sizeof(struct DATA_SOCKET_RECEIVE));

  handler_thread_maestro.mutex = &mutex;
  handler_thread_gps.mutex = &mutex;
  handler_thread_cv7.mutex = &mutex;
  handler_thread_maestro.run_threads = run_threads;
  handler_thread_gps.run_threads = run_threads;
  handler_thread_cv7.run_threads = run_threads;
  handler_thread_maestro.data_socket_receive = data_socket_receive;
  handler_thread_gps.data_socket_receive = data_socket_receive;
  handler_thread_cv7.data_socket_receive = data_socket_receive;
  handler_thread_maestro.data_socket_send = data_socket_send;
  handler_thread_gps.data_socket_send = data_socket_send;
  handler_thread_cv7.data_socket_send = data_socket_send;

  sem_wait(handler_shm.sem);
  handler_shm.shdata->shdata_arduino.address_arduino = 0x07;
  handler_shm.shdata->shdata_arduino.battery_msb = 0;
  handler_shm.shdata->shdata_arduino.battery_lsb = 0;
  handler_shm.shdata->shdata_arduino.pressure_msb = 0;
  handler_shm.shdata->shdata_arduino.pressure_lsb = 0;
  handler_shm.shdata->shdata_arduino.rudder_msb = 0;
  handler_shm.shdata->shdata_arduino.rudder_lsb = 0;
  handler_shm.shdata->shdata_arduino.sheet_msb = 0;
  handler_shm.shdata->shdata_arduino.sheet_lsb = 0;
  handler_shm.shdata->shdata_arduino.flag = 0;
  memset(&handler_shm.shdata->shdata_compass,0,sizeof(struct SHDATA_COMP));
  handler_shm.shdata->shdata_compass.address_compass = 0x19;
  sem_post(handler_shm.sem);



  printf("Creating gps_thread\n");
  if (pthread_create(&thread_gps, NULL, gps_thread, (void*) &handler_thread_gps) != 0)
      exit(EXIT_FAILURE);

  printf("Creating cv7_thread\n");
  if (pthread_create(&thread_cv7, NULL, cv7_thread, (void*) &handler_thread_cv7) != 0)
      exit(EXIT_FAILURE);


  printf("Creating maestro_thread\n");
  if (pthread_create(&thread_maestro, NULL, maestro_thread, (void*) &handler_thread_maestro) != 0)
      exit(EXIT_FAILURE);

  /*while(true){

  }*/

  pthread_join(thread_gps, NULL);
  pthread_join(thread_maestro, NULL);
  pthread_join(thread_cv7, NULL);

  return 0;
}
