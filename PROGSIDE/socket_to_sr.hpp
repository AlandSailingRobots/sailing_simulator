#ifndef SOCKET_TO_SR
#define SOCKET_TO_SR

#include <pthread.h>
#include <stdint.h>

//-----------------------------------------------------------------------------
struct DATA_SOCKET_RECEIVE{

  //=========================
  int64_t latitude=0;
  int64_t longitude=0;
  int64_t course_real = 0;
  int64_t course_magn = 0;
  int64_t speed_knot = 0;

  //=========================
  int64_t windDirection = 120;
  int64_t windSpeed = 2.1;
  int64_t windTemperature = 24;

  //=========================
  uint16_t pressure;
  uint16_t rudder;
  uint16_t sheet;
  uint16_t battery;
  uint8_t address_arduino;
  uint8_t flag_arduino;

  //=========================

  uint16_t headingVector[3];
  uint16_t magVector[3];
  uint16_t tiltVector[3];
  uint16_t accelVector[3];
  uint8_t address_compass;
  uint8_t flag_compass;
};

struct DATA_SOCKET_SEND{
  uint16_t rudder_command;
  uint16_t sheet_command;
};
//-----------------------------------------------------------------------------
struct THREAD_HANDLER {
  pthread_mutex_t *mutex;
  int *run_threads;
  struct DATA_SOCKET_SEND *data_socket_send;
  struct DATA_SOCKET_RECEIVE *data_socket_receive;
};

#endif
