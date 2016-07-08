#include <string.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <stdlib.h>

#include "maestro.h"


//-----------------------------------------------------------------------------
MaestroExchange::MaestroExchange(): state(0){}

void MaestroExchange::read_buffer(void)
{

  if (!state){
    unsigned char character = buffer_stream[0];
    buffer_stream.erase(buffer_stream.begin());
    while (buffer_stream.size()>0)
    {
       if(character == SET_SPEED || character == SET_ACCELERATION ||
          character == SET_POSITION || character == SET_POSITION_HOME ||
          character == GET_POSITION || character == GET_MOVING_STATE ||
          character == GET_ERROR)
       {
        state = 1;
        command = character;
        break;
       }
       else{
         if (!buffer_stream.size())
            break;
         character = buffer_stream[0];
         buffer_stream.erase(buffer_stream.begin());
       }

    }
  }
  else if(state=1){
    channel = buffer_stream[0];
    buffer_stream.erase(buffer_stream.begin());
    state=2;
  }
  else if(state=2){
    value1 = buffer_stream[0];
    buffer_stream.erase(buffer_stream.begin());
    state=3;
  }
  else if(state=3){
    value1 = buffer_stream[0];
    buffer_stream.erase(buffer_stream.begin());
    state=4;
  }
  else if(state=3){
    execute_command();
    state=0;
  }
  if(state>0)
      read_buffer();
  else if (buffer_stream.size()){
    read_buffer();
  }
}

void MaestroExchange::add_buffer(char *new_buffer){

  buffer_stream += new_buffer;
}

void MaestroExchange::execute_command(void){
  switch (command) {
    case SET_SPEED:
      break;
    case SET_ACCELERATION:
      break;
    case SET_POSITION:
      break;
    case SET_POSITION_HOME:
      break;
    case GET_POSITION:
      break;
    case GET_MOVING_STATE:
      break;
    case GET_ERROR:
      break;
  default:
    std::cout<<"Error in Parsing"<<std::endl;
 }
}
