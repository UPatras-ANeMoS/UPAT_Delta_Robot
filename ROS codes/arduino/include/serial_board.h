#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <stdlib.h>
#include <termios.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdbool.h>
#include <sys/poll.h>
#include <sys/ioctl.h>
#include <iostream>
#include <string>
#include <time.h>

class cserial_board
{
	//--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cGenericTool.
    cserial_board(char* port,int speed, int bytes_IN, int bytes_OUT);

    //! Destructor of cGenericTool.
    ~cserial_board();

    //--------------------------------------------------------------------------
    // PUBLIC METHODS 
    //--------------------------------------------------------------------------

    bool cserial_open();
    bool cserial_read();
    bool cserial_write();
    void cserial_close();

    //--------------------------------------------------------------------------
    // PUBLIC VARIABLES 
    //--------------------------------------------------------------------------
    unsigned char *ch_in; ///matrix to store serial buffer bytes
	unsigned char *ch_out; ///matrix that contains bytes to send over serial

    //--------------------------------------------------------------------------
    // PRIVATE METHODS 
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    // PRIVATE VARIABLES 
    //--------------------------------------------------------------------------

protected:
	int *fd; //serial port descriptor
	char *serial_address; ///serial port physical address
	int  *speed; ///serial port speed
	uint32_t *SIZE_IN; ///number of bytes per packet
	uint32_t *SIZE_OUT; ///number of bytes per packet
	struct pollfd* poll_fd; ///UNUSED --- used to poll serial for I/O events (reduces CPU load)
};
