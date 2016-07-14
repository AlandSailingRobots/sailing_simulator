#ifndef PTMX_H
#define PTMX_H

#include <termios.h>

struct PTMX
{
    int fd;
    char *port;
    struct termios origintio;
};

#ifdef __cplusplus
extern "C" {
#endif

int ptmxclose(struct PTMX *ptmx);
int ptmxopen(struct PTMX *ptmx);
int ptmxopen_speed(struct PTMX *ptmx,int baudrate);

#ifdef __cplusplus
}
#endif


#endif
