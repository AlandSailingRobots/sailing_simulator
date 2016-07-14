#ifndef SOCKET_TO_SR
#define SOCKET_TO_SR

#include <pthread.h>
#include <stdint.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include "ptmx.h"

//-----------------------------------------------------------------------------
struct DATA_SOCKET_RECEIVE{

  //=========================
  float latitude=0;
  float longitude=0;
  float course_real = 0;
  float course_magn = 0;
  float speed_knot = 0;

  //=========================
  float windDirection = 120;
  float windSpeed = 2.1;
  float windTemperature = 24;

  //=========================
  uint16_t pressure;
  uint16_t rudder;
  uint16_t sheet;
  uint16_t battery;


  //=========================

  uint16_t headingVector[3];
  uint16_t magVector[3];
  uint16_t tiltVector[3];
  uint16_t accelVector[3];
  uint8_t address_compass;
  uint8_t address_arduino;
}__attribute__((packed));

struct DATA_SOCKET_SEND{
  uint16_t rudder_command;
  uint16_t sheet_command;
}__attribute__((packed));
//-----------------------------------------------------------------------------
struct THREAD_HANDLER {
  pthread_mutex_t *mutex;
  int *run_threads;
  struct DATA_SOCKET_SEND *data_socket_send;
  struct DATA_SOCKET_RECEIVE *data_socket_receive;
  struct PTMX *serial_port;
  int *pipe;
};

struct HANDLERS_SOCKET
{
    int sockfd;
    struct sockaddr_in info_me;
};

#endif
