#ifndef MAESTRO_H
#define MAESTRO_H

#include <iostream>
#include <stdint.h>

//-----------------------------------------------------------------------------
#define SET_SPEED            0x87
#define SET_ACCELERATION     0x89
#define SET_POSITION         0x84
#define SET_POSITION_HOME    0xA2
#define GET_POSITION         0x90
#define GET_MOVING_STATE     0x93
#define GET_ERROR            0xA1

/*enum Commands : unsigned char {SET_SPEED = 0x87,SET_ACCELERATION = 0x89,
	SET_POSITION = 0x84, SET_POSITION_HOME = 0xA2,GET_POSITION = 0x90
  GET_MOVING_STATE = x93,GET_ERROR = 0xA1};*/


//---------------
class MaestroExchange {
public:
  MaestroExchange();
  ~MaestroExchange(){};

  void read_buffer();
  void add_buffer(char *new_buffer);
  void execute_command(void);
  int get_rudder_command();
  int get_sheet_command();

private:
  int position;
  int position_command;
  int speed;
  std::string buffer_stream;
  int state; //state=0 no order, =1 waiting second byte, =2 waiting third byte, =3 waiting forth byte, =4 execute command
  int channel;
  int command;
  uint8_t value1;
  uint8_t value2;
  int rudder_command;
  int sheet_command;
};
//-----------------------------------------------------------------------------

#endif
