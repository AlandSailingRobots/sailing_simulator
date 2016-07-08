
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <iostream>
#include <cstring>

#include "handler.h"

extern "C" {


int wiringPiI2CSetup(const int m_address){
  return hndopen();
}

int wiringPiI2CRead(int m_fd){
  //ONLY USED BY COMPASS DEVICE;
  return readOneCompassByte();
}

int wiringPiI2CWrite (int fd, int data){
  //ONLY USED BY COMPASS DEVICE
  writeCommand(data);
  return 1;
}

}

int wiringPiI2CReadBlock(int fd,uint8_t  *block)
{
  readBlocks(block);

  return 1;
}
