#include "ros/ros.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include <sstream>
#include "serial_board.h"
#include <inttypes.h>
#include <time.h>

#define SPEED B115200
#define BIN 14
#define BOUT 1
#define PORT "/dev/ttyACM0"
#define RATE 500 //Hz
uint32_t counter;
cserial_board* myserial;

using namespace std;
char f1[5],f2[5],f3[5];
double theta1,theta2, theta3;
double V1max = 1.533; double V2max = 1.6324; double V3max = 1.638;
double V1min = 0.81; double V2min = 0.894; double V3min = 0.92;




int main(int argc, char **argv){
	counter=0;
	//INIT ROS
	ros::init(argc, argv, "serial_publisher");
	//INIT ROS HANDLER
	ros::NodeHandle n;
	//INIT PUBLISHING FUNCTION
	std_msgs::Float64MultiArray* anadrash = new std_msgs::Float64MultiArray[3];
	ros::Publisher chatter_pub = n.advertise<std_msgs::Float64MultiArray>("feedback", 1);
// 	//INIT LISTENING FUNCTION
// 	ros::NodeHandle m;
// 	ros::Subscriber listen_arr = m.subscribe("board_commands",1,chatterCallback);
	//DEFINE PROGRAM RATE
	ros::Rate loop_rate(RATE);	
	//INIT SERIAL PORT
	char* port=new char[20];
	strcpy(port,PORT);
	myserial=new cserial_board(port, SPEED, BIN, BOUT);
	
	
	//OPEN SERIAL PORT
	if (myserial->cserial_open()){
		ROS_INFO("Board connected");
 		//START LOOP
		while (ros::ok()){	
			//Do this.
			ros::spinOnce();
			if(myserial->cserial_write() && myserial->cserial_read() && myserial->ch_in[0]=='!' && myserial->ch_in[BIN-1]=='$'){
				for (unsigned int i = 0; i < 4; i++){
					f1[i] = myserial->ch_in[i + 1];
					f2[i] = myserial->ch_in[i + 5];
					f3[i] = myserial->ch_in[i + 9];
				}
				f1[4] = '\0'; f2[4] = '\0'; f3[4] = '\0';
				double A1 = atof(f1); double A2 = atof(f2); double A3 = atof(f3);
				//cout<<A1<<"\t"<<A2<<"\t"<<A3<<endl;
				double V1 = (A1 * 5) / 1023; double V2 = (A2 * 5) / 1023; double V3 = (A3 * 5) / 1023;
				//cout<<V1<<"\t"<<V2<<"\t"<<V3<<endl;
				theta1 = (90 * (V1 - V1min)) / (V1max - V1min);
				theta2 = (90 * (V2 - V2min)) / (V2max - V2min);
				theta3 = (90 * (V3 - V3min)) / (V3max - V3min);
				if (theta1 < 0){ theta1 = 0; }
				if (theta2 < 0){ theta2 = 0; }
				if (theta3 < 0){ theta3 = 0; }
				if (theta1 > 90){ theta1 = 90; }
				if (theta2 > 90){ theta2 = 90; }
				if (theta3 > 90){ theta3 = 90; }
				
				//Clear anadrash
				anadrash->data.clear();
				uint8_t i=0;
				while(i<BIN){
					//cout<< (char) myserial->ch_in[i] << " ";
					//cout<<int (myserial->ch_in[i])<<"\t";
					printf("%c ",myserial->ch_in[i]);
					i++;
				}
				cout<<endl;
				cout<<theta1<<"\t"<<theta2<<"\t"<<theta3<<endl;
				anadrash->data.push_back(theta1); anadrash->data.push_back(theta2); anadrash->data.push_back(theta3);
				chatter_pub.publish(*anadrash);
				//Let the world know
				ROS_INFO("I published something!\n");
			}
			else{
				ROS_INFO("error read");
			}
			//ROS_INFO("%d",counter);
			//Added a delay so not to spam
			loop_rate.sleep();	
		}
		//write close main switch
			
		myserial->cserial_close();
		return 0;
	}
	else{
		ROS_INFO("Problem opening port");
		return(-1);
	}
	
}
