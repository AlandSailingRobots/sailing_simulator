#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <iostream>
#include <fstream>

#include "i2c_backend/handler.h"
#include "cv7serial.h"
#include "maestro.h"
#include "nmea.h"
#include "ptmx.h"

#include "socket_to_sr.hpp"
#include "thread_serial.h"


//-----------------------------------------------------------------------------

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
  ptmxclose(handler_thread_cv7.serial_port);
  ptmxclose(handler_thread_gps.serial_port);
  ptmxclose(handler_thread_maestro.serial_port);
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

int init_socket(int port, struct HANDLERS_SOCKET * handlers)
{
    // init socket
    handlers->sockfd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (handlers->sockfd == -1)
    {
        perror("socket");
        return -1;
    }

    // configure
    bzero((char *)&handlers->info_me, sizeof(handlers->info_me));
    handlers->info_me.sin_family = AF_INET;
    handlers->info_me.sin_port = htons(port);
    handlers->info_me.sin_addr.s_addr = htonl(INADDR_ANY);

    if( bind(handlers->sockfd, (struct sockaddr*) &handlers->info_me,
             sizeof(handlers->info_me)) == -1)
    {
        perror("bind");
        return -1;
    }

    if (listen(handlers->sockfd,1) < 0)
    {
        perror("listen");
        return -1;
    }
    return 0;
}


int main(int argc,char **argv){

  char *pathname;

  if (argc==2){
    pathname = argv[1];
  }
  else{
    pathname=(char*)"port_path.sh";
  }

  std::ofstream myfile;
  printf("Writing Port in %s\n",pathname);
  myfile.open(pathname);

  struct PTMX ptmx_cv7;
  struct PTMX ptmx_gps;
  struct PTMX ptmx_maestro;

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

  myfile << "export CV7_PORT_SIMU="<<ptmx_cv7.port<<"\n";
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

  myfile << "export GPS_PORT_SIMU="<<ptmx_gps.port<<"\n";

  // open virtual serial port
  if ( ptmxopen(&ptmx_maestro) == -1)
  {
      fprintf(stderr, "Error on ptmxopen() for maestro: %s\n", strerror(errno));
      exit(EXIT_FAILURE);
  }
  // print
  printf("PTTY maestro: %s\n", ptmx_maestro.port);



  myfile << "export MAESTRO_PORT_SIMU="<<ptmx_maestro.port<<"\n";
  myfile.close();

  // init socket
  struct HANDLERS_SOCKET handler_socket_server;
  if (init_socket(6400, &handler_socket_server) == -1)
     exit(EXIT_FAILURE);

  struct HANDLERS_SOCKET handler_socket_client;

  printf("Size of float on this platform %d\n",(int)sizeof(float)*8 );

  printf("Address: %s Port: %d\n", inet_ntoa(handler_socket_server.info_me.sin_addr),6400);
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

  struct DATA_SOCKET_SEND temp_data_sock_send;
  struct DATA_SOCKET_RECEIVE temp_data_sock_receive;
  struct DATA_SOCKET_RECEIVE dump_data_sock_receive;

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
  handler_thread_maestro.serial_port = &ptmx_maestro;
  handler_thread_gps.serial_port = &ptmx_gps;
  handler_thread_cv7.serial_port = &ptmx_cv7;
  handler_thread_maestro.pipe = maestro_pipe;


  // init shared_memory
  sem_wait(handler_shm.sem);
  memset(&handler_shm.shdata->shdata_arduino,0,sizeof(struct SHDATA_ARDU));
  handler_shm.shdata->shdata_arduino.address_arduino = 0x07;
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

  //================
  unsigned int clntLen;            /* Length of client address data structure */
  /* Set the size of the in-out parameter */
  clntLen = sizeof(handler_socket_client.info_me);
  /* Wait for a client to connect */

  printf("Waiting for simulation client...\n");
  fflush(stdout);

  if ((handler_socket_client.sockfd = accept(handler_socket_server.sockfd, (struct sockaddr *) &(handler_socket_client.info_me),
           &clntLen)) < 0)
  {
        perror("accept() failed");
  }

  // Transform socjet to on-blockig
  fcntl(handler_socket_client.sockfd, F_SETFL, O_NONBLOCK);
  fcntl(handler_socket_server.sockfd, F_SETFL, O_NONBLOCK);
  printf("Handling Simulation client %s\n", inet_ntoa(handler_socket_client.info_me.sin_addr));

  int bytes_received=0;
  while(true){
    //receive socket from simulation
    bytes_received += read(handler_socket_client.sockfd,&temp_data_sock_receive+bytes_received,sizeof(struct DATA_SOCKET_RECEIVE)-bytes_received);
    if (bytes_received==sizeof(struct DATA_SOCKET_RECEIVE)){
       pthread_mutex_lock(&mutex);
       memcpy(data_socket_receive, &temp_data_sock_receive, sizeof(struct DATA_SOCKET_RECEIVE));
       pthread_mutex_unlock(&mutex);
       //printf("sure received %d\n",bytes_received);
       bytes_received=0;
       while(read(handler_socket_client.sockfd,&dump_data_sock_receive,sizeof(struct DATA_SOCKET_RECEIVE))>0){};
    }
    if (bytes_received==-1)
      bytes_received=0;

    //printf("received %d expecting %d address_ar: %d\n",bytes_received,(int)sizeof(struct DATA_SOCKET_RECEIVE),temp_data_sock_receive.address_compass);
    //fflush(stdout);

    pthread_mutex_lock(&mutex);
    memcpy(&temp_data_sock_send,data_socket_send, sizeof(struct DATA_SOCKET_SEND));
    pthread_mutex_unlock(&mutex);
    write(handler_socket_client.sockfd,&temp_data_sock_send, sizeof(struct DATA_SOCKET_SEND));

    sem_wait(handler_shm.sem);// writing to shared memory
    handler_shm.shdata->shdata_arduino.address_arduino = 0x07;
    handler_shm.shdata->shdata_arduino.battery_msb = ((temp_data_sock_receive.battery >> 8) & 0xff);
    handler_shm.shdata->shdata_arduino.battery_lsb = ((temp_data_sock_receive.battery >> 0) & 0xff);
    handler_shm.shdata->shdata_arduino.pressure_msb = ((temp_data_sock_receive.pressure >> 8) & 0xff);
    handler_shm.shdata->shdata_arduino.pressure_lsb = ((temp_data_sock_receive.pressure >> 0) & 0xff);
    handler_shm.shdata->shdata_arduino.rudder_msb = ((temp_data_sock_receive.rudder >> 8) & 0xff);
    handler_shm.shdata->shdata_arduino.rudder_lsb = ((temp_data_sock_receive.rudder >> 0) & 0xff);
    handler_shm.shdata->shdata_arduino.sheet_msb = ((temp_data_sock_receive.sheet >> 8) & 0xff);
    handler_shm.shdata->shdata_arduino.sheet_lsb = ((temp_data_sock_receive.sheet >> 0) & 0xff);


    handler_shm.shdata->shdata_compass.address_compass = 0x19;
    handler_shm.shdata->shdata_compass.flag = 0;
    handler_shm.shdata->shdata_compass.headingVector[0] = ((temp_data_sock_receive.headingVector[0] >> 8) & 0xff);
    handler_shm.shdata->shdata_compass.headingVector[1] = ((temp_data_sock_receive.headingVector[0] >> 0) & 0xff);
    handler_shm.shdata->shdata_compass.headingVector[2] = ((temp_data_sock_receive.headingVector[1] >> 8) & 0xff);
    handler_shm.shdata->shdata_compass.headingVector[3] = ((temp_data_sock_receive.headingVector[1] >> 0) & 0xff);
    handler_shm.shdata->shdata_compass.headingVector[4] = ((temp_data_sock_receive.headingVector[2] >> 8) & 0xff);
    handler_shm.shdata->shdata_compass.headingVector[5] = ((temp_data_sock_receive.headingVector[2] >> 0) & 0xff);

    handler_shm.shdata->shdata_compass.magVector[0] = ((temp_data_sock_receive.magVector[0] >> 8) & 0xff);
    handler_shm.shdata->shdata_compass.magVector[1] = ((temp_data_sock_receive.magVector[0] >> 0) & 0xff);
    handler_shm.shdata->shdata_compass.magVector[2] = ((temp_data_sock_receive.magVector[1] >> 8) & 0xff);
    handler_shm.shdata->shdata_compass.magVector[3] = ((temp_data_sock_receive.magVector[1] >> 0) & 0xff);
    handler_shm.shdata->shdata_compass.magVector[4] = ((temp_data_sock_receive.magVector[2] >> 8) & 0xff);
    handler_shm.shdata->shdata_compass.magVector[5] = ((temp_data_sock_receive.magVector[2] >> 0) & 0xff);

    handler_shm.shdata->shdata_compass.tiltVector[0] = ((temp_data_sock_receive.tiltVector[0] >> 8) & 0xff);
    handler_shm.shdata->shdata_compass.tiltVector[1] = ((temp_data_sock_receive.tiltVector[0] >> 0) & 0xff);
    handler_shm.shdata->shdata_compass.tiltVector[2] = ((temp_data_sock_receive.tiltVector[1] >> 8) & 0xff);
    handler_shm.shdata->shdata_compass.tiltVector[3] = ((temp_data_sock_receive.tiltVector[1] >> 0) & 0xff);
    handler_shm.shdata->shdata_compass.tiltVector[4] = ((temp_data_sock_receive.tiltVector[2] >> 8) & 0xff);
    handler_shm.shdata->shdata_compass.tiltVector[5] = ((temp_data_sock_receive.tiltVector[2] >> 0) & 0xff);

    handler_shm.shdata->shdata_compass.accelVector[0] = ((temp_data_sock_receive.accelVector[0] >> 8) & 0xff);
    handler_shm.shdata->shdata_compass.accelVector[1] = ((temp_data_sock_receive.accelVector[0] >> 0) & 0xff);
    handler_shm.shdata->shdata_compass.accelVector[2] = ((temp_data_sock_receive.accelVector[1] >> 8) & 0xff);
    handler_shm.shdata->shdata_compass.accelVector[3] = ((temp_data_sock_receive.accelVector[1] >> 0) & 0xff);
    handler_shm.shdata->shdata_compass.accelVector[4] = ((temp_data_sock_receive.accelVector[2] >> 8) & 0xff);
    handler_shm.shdata->shdata_compass.accelVector[5] = ((temp_data_sock_receive.accelVector[2] >> 0) & 0xff);
    sem_post(handler_shm.sem);

    usleep(500000);
  }

  pthread_join(thread_gps, NULL);
  pthread_join(thread_maestro, NULL);
  pthread_join(thread_cv7, NULL);

  return 0;
}
