//#include "../include/serial_board.h"
#include "../include/serial_board.h"

//#define SPEED B115200


using namespace std;

cserial_board::cserial_board(char* port,int tspeed, int bytes_IN, int bytes_OUT)
{
	fd=new int[1];
	SIZE_IN=new uint32_t[1];
	*SIZE_IN=(uint32_t) bytes_IN;
	SIZE_OUT=new uint32_t[1];
	*SIZE_OUT=(uint32_t) bytes_OUT;
	speed=new int[1];
	*speed=tspeed;
	serial_address=new char[50];
	strcpy(serial_address,port);
	poll_fd=(struct pollfd*) calloc(1,sizeof(struct pollfd));
	ch_in=new unsigned char [(*SIZE_IN)+1];
	ch_out=new unsigned char [(*SIZE_OUT)+1];
}

bool cserial_board::cserial_open(){
	
// 	struct termios options;
// 	tcgetattr(*fd, &options);
// 	cfsetispeed(&options, B115200);
// 	cfsetospeed(&options, B115200);
// 	tcsetattr(*fd, TCSANOW, &options);
	//cout<<"serial opened"<<endl;
	*fd=open(serial_address, O_RDWR | O_NOCTTY );//| O_NONBLOCK);
	if (!(*fd)){
		*fd=open(serial_address, O_RDWR| O_NOCTTY);// | O_NONBLOCK);
		if (!(*fd)){
			cout<< "Problem opening serial port"<<endl;
			
		}
	}

	if (*fd>0){
		cout<< "serial opened"<<endl;
		struct termios options;
		//sleep(2); //required? to make flush work, for some reason
		//tcflush(*fd,TCIOFLUSH);	
		//options.c_cflag= *speed | CRTSCTS | CS8 | CLOCAL | CREAD;
		//options.c_iflag= IGNPAR | ICRNL;
		//options.c_oflag= 0;
		//options.c_lflag= ICANON;
		//options.c_cc[VMIN]= *SIZE_IN-1;//1
		//options.c_cc[VTIME]= 0;
		
		
		
		
		/* 115200 baud */
		tcgetattr(*fd, &options);
		cfsetispeed(&options, B115200);
		cfsetospeed(&options, B115200);
		/* 8 bits, no parity, no stop bits */
		options.c_cflag &= ~PARENB;
		options.c_cflag &= ~CSTOPB;
		options.c_cflag &= ~CSIZE;
		options.c_cflag |= CS8;
		/* no hardware flow control */
		options.c_cflag &= ~CRTSCTS;
		/* enable receiver, ignore status lines */
 		options.c_cflag |= CREAD | CLOCAL;
		/* disable input/output flow control, disable restart chars */
		options.c_iflag = ~(IXON | IXOFF | IXANY);
		/* disable canonical input, disable echo,
		disable visually erase chars,
		disable terminal-generated signals */
		options.c_lflag = ~(ICANON | ECHO | ECHOE | ISIG);
		/* disable output processing */
		options.c_oflag = ~OPOST;
		options.c_cc[VTIME]=10; //inter-character timer 1->0.1s
		options.c_cc[VMIN]=*SIZE_IN; //blocks read until VMIN bytes received
		tcsetattr(*fd, TCSANOW, &options);
		sleep(2);
		tcflush(*fd,TCIOFLUSH);	
		
		//tcsetattr(*fd, TCSANOW, &options);
		//poll_fd->fd=*fd;
		//poll_fd->events=POLLIN;
	return true;
	}
	else{
	return false;
	}
}

bool cserial_board::cserial_read(){
	bool ret=false;
	      if(read(*fd,ch_in,*SIZE_IN)==*SIZE_IN){
	//int len=read(*fd,ch_in,*SIZE_IN);
	//if(len>=1){
		      //if (1){
			      //(ch_in)[*SIZE_IN-2]=='$' && (ch_in)[*SIZE_IN-15]=='!'
			      ret=true;
			      //cout<< "read OK "<< endl;
		      //}
		      //else{
			      //for(len=0; len<*SIZE_IN;len++)
			      //cout<<(int) (ch_in)[len] <<" " ;
			      //cout << endl;
			      //cout<<"flush" <<endl;
			      //tcflush(*fd,TCIFLUSH);
		     // }
	      }
	      else{
		      cout<< "read Problem" << endl;
	      }
	
	return ret;
}

bool cserial_board::cserial_write(){
	if (write(*fd,ch_out,*SIZE_OUT)==*SIZE_OUT){
		return true;
	}
	else{
		cout<< "Data write problem"<<endl;		
	}
	return false;
}

void cserial_board::cserial_close(){
	tcflush(*fd,TCIFLUSH);	
	close(*fd);
	//cout << "serial closed" << endl;
}

cserial_board::~cserial_board(){
	this->cserial_close();
	delete fd;
	delete SIZE_IN;
	delete SIZE_OUT;
	delete speed;
	delete serial_address;
	delete poll_fd;
	delete ch_in;
	delete ch_out;
}
