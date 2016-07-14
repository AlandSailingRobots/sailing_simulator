#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>

#include "i2c_backend/handler.h"
#include "cv7serial.h"
#include "maestro.h"
#include "nmea.h"
#include "ptmx.h"

#include "socket_to_sr.hpp"


#define MAX(a,b) ((a) > (b) ? a : b)

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
  struct PTMX *ptmx_gps_thread = th_handler->serial_port;

  pthread_mutex_lock(th_handler->mutex);
  run_thread = *(th_handler->run_threads);
  pthread_mutex_unlock(th_handler->mutex);

  while(run_thread){
    pthread_mutex_lock(th_handler->mutex);
    run_thread = *(th_handler->run_threads);
    pthread_mutex_unlock(th_handler->mutex);
    write_rmc(ptmx_gps_thread->fd, th_handler);
    sleep(1);
  }

  printf("GPS exiting\n");
  fflush(stdout);
  return NULL;
}

void *cv7_thread(void *thread_handler){
  int run_thread;
  struct THREAD_HANDLER *th_handler = (struct THREAD_HANDLER*) thread_handler;
  struct PTMX *ptmx_cv7_thread = th_handler->serial_port;

  pthread_mutex_lock(th_handler->mutex);
  run_thread = *(th_handler->run_threads);
  pthread_mutex_unlock(th_handler->mutex);

  while(run_thread){
    pthread_mutex_lock(th_handler->mutex);
    run_thread = *(th_handler->run_threads);
    pthread_mutex_unlock(th_handler->mutex);
    write_cv7(ptmx_cv7_thread->fd, th_handler);
    sleep(1);
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
  struct PTMX *ptmx_maestro_thread = th_handler->serial_port;
  int *maestro_pipe = th_handler->pipe;

  pthread_mutex_lock(th_handler->mutex);
  run_thread = *(th_handler->run_threads);
  pthread_mutex_unlock(th_handler->mutex);

  while(run_thread){

    FD_ZERO(&pdset);
    FD_SET(ptmx_maestro_thread->fd, &pdset);
    FD_SET(maestro_pipe[0], &pdset);
    int max = MAX(ptmx_maestro_thread->fd,maestro_pipe[0]);
    select(max+1, &pdset, NULL, NULL, NULL);

    if (FD_ISSET(maestro_pipe[0], &pdset))
    {
     break;
    }
    unsigned char buffer[200];
    memset(buffer, '\0',200);
    int n = read(ptmx_maestro_thread->fd, buffer, sizeof(buffer));
    int res = 1;
    if (n < 0){
      fputs("Maestro progside read failed!\n", stderr);
      printf("buffer maestro %s",buffer);
      usleep(500000);
    }
    else{
      res = maestro_handler.add_buffer(buffer);
    }

    if (!res){ //received get error command
       unsigned short dataHandle = 0;
       printf("send dataHandle\n");
       write(ptmx_maestro_thread->fd, &dataHandle, sizeof(dataHandle));
    }

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
