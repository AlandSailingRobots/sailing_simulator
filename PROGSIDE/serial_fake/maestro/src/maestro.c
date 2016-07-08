//-----------------------------------------------------------------------------
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>

#include <ptmx.h>
#include <cv7serial.h>

//-----------------------------------------------------------------------------
struct PTMX ptmx;


//-----------------------------------------------------------------------------
void signals_handler(int signal_number)
{
    printf("Signal catched.\n");
    ptmxclose(&ptmx);
    exit(EXIT_SUCCESS);
}

//-----------------------------------------------------------------------------
int write_cv7(int fd)
{
    size_t bytes = 0;

    struct CV7PHRASE cv7phrase;
    cv7phrase.windDirection = 120;
    cv7phrase.windSpeed = 2.1;
    cv7phrase.windTemperature = 24;

    if ( cv7_phrase(&cv7phrase) == -1)
    {
        perror("cv7_phrase");
        exit(EXIT_FAILURE);
    }
    else
        bytes = write(fd, cv7phrase.frame, sizeof(cv7phrase.frame));


    return bytes;
}

//-----------------------------------------------------------------------------
int main()
{
    // open virtual serial port
    if ( ptmxopen(&ptmx) == -1)
    {
        fprintf(stderr, "Error on ptmxopen(): %s\n", strerror(errno));
        exit(EXIT_FAILURE);
    }

    // signals handler
    struct sigaction action;
    action.sa_handler = signals_handler;
    sigemptyset(& (action.sa_mask));
    action.sa_flags = 0;
    sigaction(SIGINT, & action, NULL);

    // print
    printf("PTTY: %s\n", ptmx.port);

    // work
    while(1)
    {
        //sleep(2);
        write_cv7(ptmx.fd);
        sleep(1);
    }

    ptmxclose(&ptmx);

    return EXIT_SUCCESS;
}
