#include <string.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <stdlib.h>

#include "maestro.h"


//-----------------------------------------------------------------------------
MaestroExchange::MaestroExchange(): state(0),
  rudder_command(0),
  sheet_command(0){}

int MaestroExchange::read_buffer(void)
{
  int retValue=1;
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
  else if(state==1){// Ruder=4, Sheet=3
    channel = buffer_stream[0];
    buffer_stream.erase(buffer_stream.begin());
    state=2;
  }
  else if(state==2){
    value1 = buffer_stream[0];
    buffer_stream.erase(buffer_stream.begin());
    state=3;
  }
  else if(state==3){
    value2 = buffer_stream[0];
    buffer_stream.erase(buffer_stream.begin());
    retValue = execute_command();
    state=0;
  }
  if (buffer_stream.size()){
    retValue = read_buffer();
  }
  return retValue;
}

int MaestroExchange::add_buffer(unsigned char *new_buffer){
  int size_max = sizeof(new_buffer);
  int i=0;
  while(i<size_max && new_buffer[i]!='\0'){
    buffer_stream.push_back(new_buffer[i]);
    i++;
  }
  return read_buffer();
}

int MaestroExchange::execute_command(void){
  switch (command) {
    case SET_SPEED:
      break;
    case SET_ACCELERATION:
      break;
    case SET_POSITION:
      if (channel==4)
       {rudder_command=((int) value1 <<8) + (int) value2;}
      else if(channel ==3)
       {sheet_command=((int) value1 <<8) + (int) value2;}// Ruder=4, Sheet=3
      break;
    case SET_POSITION_HOME:
      break;
    case GET_POSITION:
      break;
    case GET_MOVING_STATE:
      break;
    case GET_ERROR:return 0;
      break;
  default:
    std::cout<<"Error in Parsing"<<std::endl;
 }
 return 1;
}

int MaestroExchange::get_rudder_command(){
  return rudder_command;
}
int MaestroExchange::get_sheet_command(){
  return sheet_command;
}
